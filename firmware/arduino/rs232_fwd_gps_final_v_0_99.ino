/*
 * RS232 FWD GPS v0.9.9 MADRID FINAL - ESP32-S3 HAS (UM980) + BNO085 + Dynatest FWM
 * 
 * Hardware:
 *   - GNSS (UM980):    RX=44, TX=43, BAUD=115200 (HAS Galileo)
 *   - Dynatest Output: RX=18, TX=17, BAUD=38400
 *   - IMU (BNO085):    SDA=8, SCL=9, I2C 400kHz, addr 0x4B
 *   - LED_RED:         Pin 4  (GNSS/HAS state)
 *   - LED_GREEN:       Pin 5  (Movement/Lock state)
 * 
 * State Machine:
 *   - Movement: MOVING → AVERAGING (speedMS < 0.20m/s, 2s) → LOCKED (15s avg) → MOVING (>1.0m dist)
 *   - HAS:      SIN_PPP → PPP_CONVERGING → PPP_ESTABLE
 * 
 * Antenna Offset: 0.55m RIGHT (bearing = yaw + 90°)
 * Magnetic Declination: Madrid 1.0° (convert mag north → geog north)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

// ============================================================================
// CONFIGURACIÓN DE PINES Y PARÁMETROS
// ============================================================================

#define GNSS_RX         44
#define GNSS_TX         43
#define GNSS_BAUD       115200
#define DYNATEST_RX     18
#define DYNATEST_TX     17
#define DYNATEST_BAUD   38400

#define LED_RED         4
#define LED_GREEN       5

#define I2C_SDA         8
#define I2C_SCL         9
#define I2C_FREQ        400000

#define OFFSET_M        0.55
#define DECLINATION_OFFSET 1.0
#define EARTH_RADIUS    6378137.0

#define STOP_THRESHOLD_MS      2000      // Umbral: speedMS < 0.20m/s durante 2s → AVERAGING
#define MOVING_THRESHOLD_MS    300       // Umbral entrada MOVING
#define SPEED_FRESHNESS_MS     2000
#define GGA_FRESHNESS_MS       2000
#define IMU_RETRY_MS           5000
#define AVERAGING_WINDOW_MS    15000
#define OUTPUT_PERIOD_MS       100
#define NEW_LOCATION_DIST      1.0       // m (exit LOCKED si distancia > 1.0m)
#define MAX_SAMPLES            160
#define CIRCULAR_BUFFER_SIZE   16        // Para mean_circular_deg

#define SPEED_ENTER_STOP       0.20      // m/s
#define SPEED_EXIT_STOP        0.30      // m/s

// ============================================================================
// ENUMERACIONES
// ============================================================================

enum MovementState {
  MOVING = 0,
  AVERAGING = 1,
  LOCKED = 2
};

enum PPPState {
  SIN_PPP = 0,
  PPP_CONVERGING = 1,
  PPP_ESTABLE = 2
};

// ============================================================================
// VARIABLES GLOBALES - ESTADO DE MOVIMIENTO
// ============================================================================

MovementState movementState = MOVING;
PPPState pppState = SIN_PPP;

uint32_t lastStopCheckMs = 0;      // Inicio de temporizador parada
uint32_t averagingStartMs = 0;     // Inicio de ventana promediación
uint32_t lastOutputMs = 0;         // Último TX a Dynatest

// ============================================================================
// VARIABLES GLOBALES - POSICIÓN GNSS
// ============================================================================

bool gnssValid = false;
double currentLat = 0.0;
double currentLon = 0.0;
double currentAlt = 0.0;
double currentSpeedMS = 0.0;
double currentCourseOverGround = 0.0;

uint32_t lastSpeedUpdateMs = 0;
uint32_t lastGgaMs = 0;
uint32_t lastSampledGgaMs = 0;

// ============================================================================
// VARIABLES GLOBALES - BUFFERS DE PROMEDIACIÓN
// ============================================================================

double latBuffer[MAX_SAMPLES];
double lonBuffer[MAX_SAMPLES];
double altBuffer[MAX_SAMPLES];
double yawBuffer[CIRCULAR_BUFFER_SIZE];

int sampleCount = 0;

// ============================================================================
// VARIABLES GLOBALES - POSICIÓN LOCKED
// ============================================================================

bool lockedValid = false;
double lockedLat = 0.0;
double lockedLon = 0.0;
double lockedAlt = 0.0;
double lockedYaw = 0.0;

double lastLockedLat = 0.0;
double lastLockedLon = 0.0;

// ============================================================================
// VARIABLES GLOBALES - IMU (BNO085)
// ============================================================================

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

double currentYaw = 0.0;
uint32_t lastBnoMs = 0;
bool bnoAvailable = false;

// ============================================================================
// VARIABLES GLOBALES - UART BUFFERS
// ============================================================================

static char gnssLineBuf[220];
static int gnssLineIdx = 0;

// ============================================================================
// FUNCIONES AUXILIARES - MATEMÁTICAS
// ============================================================================

double toRadians(double deg) {
  return deg * M_PI / 180.0;
}

double toDegrees(double rad) {
  return rad * 180.0 / M_PI;
}

double normalizeAngle(double ang) {
  while (ang < 0.0) ang += 360.0;
  while (ang >= 360.0) ang -= 360.0;
  return ang;
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dlat = toRadians(lat2 - lat1);
  double dlon = toRadians(lon2 - lon1);
  double a = sin(dlat/2.0) * sin(dlat/2.0) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) *
             sin(dlon/2.0) * sin(dlon/2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS * c;
}

// Circular mean para ángulos (0-360°)
double mean_circular_deg(double* buf, int cnt) {
  if (cnt <= 0) return 0.0;
  
  double sin_sum = 0.0, cos_sum = 0.0;
  for (int i = 0; i < cnt; i++) {
    double rad = toRadians(buf[i]);
    sin_sum += sin(rad);
    cos_sum += cos(rad);
  }
  double mean_rad = atan2(sin_sum / cnt, cos_sum / cnt);
  return normalizeAngle(toDegrees(mean_rad));
}

// Trimmed mean (5% outliers)
double trimmed_mean(double* buf, int cnt) {
  if (cnt <= 0) return NAN;
  
  // Copiar en buffer temporal para sort (evitamos modificar original)
  static double sortBuf[MAX_SAMPLES];
  memcpy(sortBuf, buf, cnt * sizeof(double));
  
  // Sort simple (bubble sort para 160 max)
  for (int i = 0; i < cnt - 1; i++) {
    for (int j = 0; j < cnt - i - 1; j++) {
      if (sortBuf[j] > sortBuf[j+1]) {
        double tmp = sortBuf[j];
        sortBuf[j] = sortBuf[j+1];
        sortBuf[j+1] = tmp;
      }
    }
  }
  
  // Trim 5%
  int trimCnt = (int)ceil(cnt * 0.05);
  int start = trimCnt;
  int end = cnt - trimCnt;
  
  if (start >= end) {
    start = 0;
    end = cnt;
  }
  
  double sum = 0.0;
  int validCnt = 0;
  for (int i = start; i < end; i++) {
    sum += sortBuf[i];
    validCnt++;
  }
  
  return (validCnt > 0) ? sum / validCnt : NAN;
}

// ============================================================================
// FUNCIONES UART Y PARSING
// ============================================================================

void parseGGA(const char* line) {
  // $GPGGA,hhmmss.ss,ddmm.mmmm,N/S,dddmm.mmmm,E/W,Q,satellites,HDOP,alt,M,geoid,M,age,id*checksum
  
  // Validar checksum
  const char* asterisk = strchr(line, '*');
  if (!asterisk) return;
  
  unsigned char calcChecksum = 0;
  for (const char* p = line + 1; p < asterisk; p++) {
    calcChecksum ^= *p;
  }
  
  unsigned char rxChecksum = 0;
  sscanf(asterisk + 1, "%02hhx", &rxChecksum);
  
  if (calcChecksum != rxChecksum) {
    Serial.println("[GNSS] GGA checksum error");
    return;
  }
  
  // Parse fields
  char* fields[15];
  char lineCopy[220];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';
  
  int fieldCount = 0;
  fields[0] = lineCopy;
  for (char* p = lineCopy; *p; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[++fieldCount] = p + 1;
      if (fieldCount >= 14) break;
    }
  }
  
  // Validación mínima
  if (fieldCount < 9) return;
  
  // fixQ
  int fixQ = atoi(fields[6]);
  if (fixQ < 1) return;
  
  // Lat
  const char* latStr = fields[2];
  const char* latHem = fields[3];
  if (strlen(latStr) < 7) return;
  
  int latDeg = (latStr[0] - '0') * 10 + (latStr[1] - '0');
  double latMin = strtod(latStr + 2, NULL);
  double lat = latDeg + latMin / 60.0;
  if (latHem[0] == 'S') lat = -lat;
  
  // Lon
  const char* lonStr = fields[4];
  const char* lonHem = fields[5];
  if (strlen(lonStr) < 8) return;
  
  int lonDeg = (lonStr[0] - '0') * 100 +
               (lonStr[1] - '0') * 10 +
               (lonStr[2] - '0');
  double lonMin = strtod(lonStr + 3, NULL);
  double lon = lonDeg + lonMin / 60.0;
  if (lonHem[0] == 'W') lon = -lon;
  
  // Alt
  double alt = strtod(fields[9], NULL);
  
  currentLat = lat;
  currentLon = lon;
  currentAlt = alt;
  gnssValid = true;
  lastGgaMs = millis();
  
  Serial.printf("[GNSS] GGA: lat=%.6f, lon=%.6f, alt=%.1f, fixQ=%d\n",
                lat, lon, alt, fixQ);
}

void parseRMC(const char* line) {
  // $GPRMC,hhmmss.ss,A,ddmm.mmmm,N/S,dddmm.mmmm,E/W,speed,course,ddmmyy,...*checksum
  
  // Validar checksum
  const char* asterisk = strchr(line, '*');
  if (!asterisk) return;
  
  unsigned char calcChecksum = 0;
  for (const char* p = line + 1; p < asterisk; p++) {
    calcChecksum ^= *p;
  }
  
  unsigned char rxChecksum = 0;
  sscanf(asterisk + 1, "%02hhx", &rxChecksum);
  
  if (calcChecksum != rxChecksum) return;
  
  // Parse fields
  char* fields[13];
  char lineCopy[220];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';
  
  int fieldCount = 0;
  fields[0] = lineCopy;
  for (char* p = lineCopy; *p; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[++fieldCount] = p + 1;
      if (fieldCount >= 12) break;
    }
  }
  
  if (fieldCount < 8) return;
  
  // Status (A = válido)
  if (fields[2][0] != 'A') return;
  
  // Speed (knots → m/s)
  double speedKnots = strtod(fields[7], NULL);
  currentSpeedMS = speedKnots * 0.51444;  // 1 knot ≈ 0.51444 m/s
  
  // Course Over Ground
  currentCourseOverGround = strtod(fields[8], NULL);
  
  lastSpeedUpdateMs = millis();
  
  Serial.printf("[GNSS] RMC: speed=%.2f m/s, COG=%.1f°\n",
                currentSpeedMS, currentCourseOverGround);
}

void parsePPPNAV(const char* line) {
  // #PPPNAVA,ppp_state,satcount,...
  // Buscar en línea completa: "PPP_ESTABLE", "PPP_CONVERGING", "HAS"
  
  if (strstr(line, "PPP_ESTABLE")) {
    pppState = PPP_ESTABLE;
    Serial.println("[HAS] State: PPP_ESTABLE (HAS locked)");
  } else if (strstr(line, "PPP_CONVERGING")) {
    pppState = PPP_CONVERGING;
    Serial.println("[HAS] State: PPP_CONVERGING");
  } else if (strstr(line, "HAS")) {
    pppState = PPP_CONVERGING;
    Serial.println("[HAS] State: HAS detected");
  } else {
    pppState = SIN_PPP;
  }
}

void readGNSSSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();
    
    if (c == '$' || c == '#') {
      gnssLineIdx = 0;
      gnssLineBuf[gnssLineIdx++] = c;
    } else if (gnssLineIdx > 0) {
      if (c == '\n') {
        gnssLineBuf[gnssLineIdx] = '\0';
        
        // Procesar línea
        if (strstr(gnssLineBuf, "$GPGGA")) {
          parseGGA(gnssLineBuf);
        } else if (strstr(gnssLineBuf, "$GPRMC")) {
          parseRMC(gnssLineBuf);
        } else if (strstr(gnssLineBuf, "#PPPNAVA")) {
          parsePPPNAV(gnssLineBuf);
        }
        
        gnssLineIdx = 0;
      } else if (gnssLineIdx < (int)sizeof(gnssLineBuf) - 1) {
        gnssLineBuf[gnssLineIdx++] = c;
      } else {
        gnssLineIdx = 0;  // Buffer overflow, reset
      }
    }
  }
}

// ============================================================================
// FUNCIONES IMU (BNO085)
// ============================================================================

bool initBNO085() {
  if (!bno08x.begin_I2C(0x4B, &Wire1)) {
    Serial.println("[IMU] BNO085 init failed!");
    return false;
  }
  
  // Configurar reporte de rotación vectorial (100 Hz)
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("[IMU] Could not enable rotation vector");
    return false;
  }
  
  bnoAvailable = true;
  lastBnoMs = millis();
  Serial.println("[IMU] BNO085 initialized");
  return true;
}

double readYaw() {
  uint32_t nowMs = millis();
  
  // Timeout: si no hay datos en 5s, retornar 0.0
  if (nowMs - lastBnoMs > IMU_RETRY_MS) {
    Serial.println("[IMU] Timeout - using yaw=0.0");
    return 0.0;
  }
  
  if (!bnoAvailable) return 0.0;
  
  // Verificar reset interno
  if (bno08x.wasReset()) {
    Serial.println("[IMU] Reset detected - reinitializing...");
    bnoAvailable = initBNO085();
    if (!bnoAvailable) return 0.0;
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      // Rotación vector: (i, j, k, real)
      // Yaw = atan2(2*(real*k + i*j), 1 - 2*(j*j + k*k))
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      float real = sensorValue.un.rotationVector.real;
      
      double yaw_rad = atan2(2.0 * (real * k + i * j),
                             1.0 - 2.0 * (j * j + k * k));
      double yaw_deg = toDegrees(yaw_rad);
      
      // Restar declinación magnética para convertir a norte geográfico
      yaw_deg -= DECLINATION_OFFSET;
      
      currentYaw = normalizeAngle(yaw_deg);
      lastBnoMs = nowMs;
      
      return currentYaw;
    }
  }
  
  return currentYaw;  // Retornar último valor conocido
}

// ============================================================================
// FUNCIONES LED
// ============================================================================

void updateLED() {
  uint32_t nowMs = millis();
  
  // Período de startup (0-1200ms): ambos LEDs parpadean 200ms
  static uint32_t startupMs = 0;
  if (startupMs == 0) {
    startupMs = nowMs;
  }
  
  if (nowMs - startupMs < 1200) {
    // Dual blink 200ms
    uint32_t cycle = (nowMs - startupMs) % 400;
    bool on = (cycle < 200);
    digitalWrite(LED_RED, on ? HIGH : LOW);
    digitalWrite(LED_GREEN, on ? HIGH : LOW);
    return;
  }
  
  // LED_RED: GNSS/HAS state
  if (!gnssValid) {
    digitalWrite(LED_RED, LOW);  // OFF: sin fix GNSS
  } else if (pppState == PPP_CONVERGING) {
    // Blink 200ms
    uint32_t cycle = nowMs % 400;
    digitalWrite(LED_RED, (cycle < 200) ? HIGH : LOW);
  } else {
    digitalWrite(LED_RED, HIGH);  // ON: fix adquirido
  }
  
  // LED_GREEN: Movement/Lock state
  if (movementState == MOVING) {
    digitalWrite(LED_GREEN, LOW);  // OFF: moviéndose
  } else if (movementState == AVERAGING) {
    // Blink 500ms
    uint32_t cycle = nowMs % 1000;
    digitalWrite(LED_GREEN, (cycle < 500) ? HIGH : LOW);
  } else if (movementState == LOCKED && lockedValid) {
    digitalWrite(LED_GREEN, HIGH);  // ON: locked confirmado
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}

// ============================================================================
// FUNCIONES MÁQUINA DE ESTADOS
// ============================================================================

int getOutputFixQ() {
  if (!gnssValid) return 0;
  
  if (movementState == LOCKED && lockedValid) {
    return 4;  // RTK (HAS locked)
  } else if (pppState == PPP_ESTABLE) {
    return 4;  // RTK
  } else if (pppState == PPP_CONVERGING) {
    return 2;  // Diferencial (HAS converging)
  } else {
    return 1;  // GPS base
  }
}

void updateMovementState() {
  uint32_t nowMs = millis();
  
  // Validar freshness de velocidad
  bool speedValid = (nowMs - lastSpeedUpdateMs) <= SPEED_FRESHNESS_MS;
  if (!speedValid) currentSpeedMS = 0.0;
  
  // Validar freshness de GGA
  bool ggaValid = (nowMs - lastGgaMs) <= GGA_FRESHNESS_MS;
  if (!ggaValid) gnssValid = false;
  
  switch (movementState) {
    case MOVING:
      // Si velocidad < 0.20 m/s durante 2s → inicia contador parada
      if (speedValid && currentSpeedMS < SPEED_ENTER_STOP) {
        if (lastStopCheckMs == 0) {
          lastStopCheckMs = nowMs;
        }
      } else if (speedValid && currentSpeedMS >= SPEED_EXIT_STOP) {
        lastStopCheckMs = 0;  // Reset timer
      }
      
      // Si lleva 2s lento → AVERAGING (sin STOP_CONFIRM)
      if (lastStopCheckMs > 0 && (nowMs - lastStopCheckMs) >= STOP_THRESHOLD_MS) {
        if (sampleCount == 0) {  // Primera vez en AVERAGING
          averagingStartMs = nowMs;
          sampleCount = 0;
          memset(latBuffer, 0, sizeof(latBuffer));
          memset(lonBuffer, 0, sizeof(lonBuffer));
          memset(altBuffer, 0, sizeof(altBuffer));
          memset(yawBuffer, 0, sizeof(yawBuffer));
          lastSampledGgaMs = 0;
          
          movementState = AVERAGING;
          lockedValid = false;
          Serial.println("[STATE] MOVING → AVERAGING (SIN STOP_CONFIRM)");
        }
      }
      break;
      
    case AVERAGING:
      // Sampling gate: solo si GGA nueva (1 Hz)
      if (lastGgaMs != lastSampledGgaMs && sampleCount < MAX_SAMPLES) {
        latBuffer[sampleCount] = currentLat;
        lonBuffer[sampleCount] = currentLon;
        altBuffer[sampleCount] = currentAlt;
        yawBuffer[sampleCount % CIRCULAR_BUFFER_SIZE] = currentYaw;
        sampleCount++;
        lastSampledGgaMs = lastGgaMs;
        
        Serial.printf("[SAMPLE] %d/160 - %.1fs remaining\n",
                      sampleCount, (AVERAGING_WINDOW_MS - (nowMs - averagingStartMs)) / 1000.0);
      }
      
      // Si velocidad aumenta > 0.30 m/s → vuelve a MOVING
      if (speedValid && currentSpeedMS > SPEED_EXIT_STOP) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        sampleCount = 0;
        Serial.println("[STATE] AVERAGING → MOVING (velocidad aumentó)");
        break;
      }
      
      // Si ventana completada (15s) → LOCKED
      if ((nowMs - averagingStartMs) >= AVERAGING_WINDOW_MS && sampleCount > 0) {
        // Calcular trimmed means
        lockedLat = trimmed_mean(latBuffer, sampleCount);
        lockedLon = trimmed_mean(lonBuffer, sampleCount);
        lockedAlt = trimmed_mean(altBuffer, sampleCount);
        
        // Circular mean para yaw (usar min(sampleCount, CIRCULAR_BUFFER_SIZE) muestras)
        int yawCnt = (sampleCount < CIRCULAR_BUFFER_SIZE) ? sampleCount : CIRCULAR_BUFFER_SIZE;
        lockedYaw = mean_circular_deg(yawBuffer, yawCnt);
        
        if (isnan(lockedLat) || isnan(lockedLon)) {
          movementState = MOVING;
          lockedValid = false;
          sampleCount = 0;
          Serial.println("[STATE] AVERAGING failed - invalid trimmed means");
          break;
        }
        
        // Aplicar offset UNA SOLA VEZ (hacia derecha: yaw + 90°)
        double correctionBearing = normalizeAngle(lockedYaw + 90.0);
        double offsetRad = OFFSET_M / EARTH_RADIUS;
        double bearingRad = toRadians(correctionBearing);
        
        double latOffset = offsetRad * cos(bearingRad);
        double lonOffset = offsetRad * sin(bearingRad) / cos(toRadians(lockedLat));
        
        lockedLat += latOffset;
        lockedLon += lonOffset;
        
        lastLockedLat = lockedLat;
        lastLockedLon = lockedLon;
        
        movementState = LOCKED;
        lockedValid = true;
        lastStopCheckMs = 0;
        
        Serial.printf("[STATE] AVERAGING → LOCKED\n");
        Serial.printf("[LOCKED] lat=%.6f, lon=%.6f, yaw=%.1f°, bearing=%.1f°\n",
                      lockedLat, lockedLon, lockedYaw, correctionBearing);
      }
      break;
      
    case LOCKED:
      // Si velocidad alta → vuelve a MOVING
      if (speedValid && currentSpeedMS > SPEED_EXIT_STOP) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        sampleCount = 0;
        Serial.println("[STATE] LOCKED → MOVING (velocidad alta)");
        break;
      }
      
      // Si distancia > 1.0m desde posición locked → vuelve a MOVING
      if (gnssValid) {
        double dist = haversine(currentLat, currentLon, lastLockedLat, lastLockedLon);
        if (dist > NEW_LOCATION_DIST) {
          movementState = MOVING;
          lockedValid = false;
          lastStopCheckMs = 0;
          sampleCount = 0;
          Serial.printf("[STATE] LOCKED → MOVING (distance %.2fm > 1.0m)\n", dist);
          break;
        }
      }
      break;
  }
}

void transmitDynatestOutput() {
  uint32_t nowMs = millis();
  if (nowMs - lastOutputMs < OUTPUT_PERIOD_MS) return;
  lastOutputMs = nowMs;
  
  int fixQ = getOutputFixQ();
  if (!gnssValid || fixQ < 1) return;
  
  double outLat = 0.0, outLon = 0.0, outAlt = 0.0, outYaw = 0.0;
  
  if (movementState == LOCKED && lockedValid) {
    // LOCKED: posición pre-calculada (YA tiene offset)
    outLat = lockedLat;
    outLon = lockedLon;
    outAlt = lockedAlt;
    outYaw = lockedYaw;
    
    Serial.printf("[TX-LOCKED] fixQ=%d, pos=%.6f,%.6f, yaw=%.1f°\n",
                  fixQ, outLat, outLon, outYaw);
  } else {
    // INST: posición instantánea + offset dinámico (hacia derecha: yaw + 90°)
    outLat = currentLat;
    outLon = currentLon;
    outAlt = currentAlt;
    outYaw = currentYaw;
    
    // Aplicar offset dinámico
    double correctionBearing = normalizeAngle(outYaw + 90.0);
    double offsetRad = OFFSET_M / EARTH_RADIUS;
    double bearingRad = toRadians(correctionBearing);
    
    double latOffset = offsetRad * cos(bearingRad);
    double lonOffset = offsetRad * sin(bearingRad) / cos(toRadians(outLat));
    
    outLat += latOffset;
    outLon += lonOffset;
    
    Serial.printf("[TX-INST] fixQ=%d, pos=%.6f,%.6f, yaw=%.1f°, bearing=%.1f°\n",
                  fixQ, outLat, outLon, outYaw, correctionBearing);
  }
  
  // Construir NMEA GGA con checksum
  char gga[120];
  snprintf(gga, sizeof(gga),
           "$GPGGA,120000.00,%02d%07.4f,%c,%03d%07.4f,%c,%d,08,1.0,%.1f,M,0.0,M,,",
           (int)fabs(outLat), (fabs(outLat) - (int)fabs(outLat)) * 60.0, (outLat >= 0) ? 'N' : 'S',
           (int)fabs(outLon), (fabs(outLon) - (int)fabs(outLon)) * 60.0, (outLon >= 0) ? 'E' : 'W',
           fixQ, outAlt);
  
  unsigned char checksum = 0;
  for (const char* p = gga + 1; *p && *p != '*'; p++) {
    checksum ^= *p;
  }
  
  char output[140];
  snprintf(output, sizeof(output), "%s*%02X\r\n", gga, checksum);
  
  Serial2.write((const uint8_t*)output, strlen(output));
}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {
  // Serial debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[SETUP] RS232 FWD GPS v0.9.9 MADRID");
  
  // GNSS UART (UM980 HAS)
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  Serial.println("[SETUP] GNSS UART initialized (HAS)");
  
  // Dynatest Output UART
  Serial2.begin(DYNATEST_BAUD, SERIAL_8N1, DYNATEST_RX, DYNATEST_TX);
  Serial.println("[SETUP] Dynatest Output UART initialized");
  
  // I2C para IMU
  Wire1.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  if (!initBNO085()) {
    Serial.println("[SETUP] BNO085 init failed - will retry");
  }
  
  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  Serial.println("[SETUP] LEDs initialized - dual blink starting");
  
  Serial.printf("[CONFIG] OFFSET_M=%.2f, DECL_OFFSET=%.1f°\n", OFFSET_M, DECLINATION_OFFSET);
  Serial.println("[SETUP] Ready!");
}

void loop() {
  // Leer UART GNSS (no bloqueante)
  readGNSSSerial();
  
  // Leer yaw del BNO085
  readYaw();
  
  // Actualizar máquina de estados
  updateMovementState();
  
  // Actualizar LEDs
  updateLED();
  
  // Transmitir a Dynatest
  transmitDynatestOutput();
}
