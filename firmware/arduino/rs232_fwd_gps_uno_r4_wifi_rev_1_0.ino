/*
 * RS232 FWD GPS Rev.1.0 - Arduino UNO R4 WiFi (hardware real)
 *
 * MCU principal: Renesas RA4M1
 * Conectividad integrada: ESP32-S3 solo para Wi-Fi/Bluetooth (no usada aquí)
 *
 * Enlaces reales en UNO R4 WiFi:
 *   - Serial  : USB CDC. Se usa para diagnóstico o como salida Dynatest limpia,
 *               pero no para ambos al mismo tiempo.
 *   - Serial1 : UART hardware en D0/RX y D1/TX para GNSS UM980 a 115200 baudios.
 *   - Wire    : I2C principal en SDA/SCL a 100 kHz para BNO085.
 *
 * Pines elegidos para LEDs externos (evitan UART/I2C/CAN):
 *   - LED1_PIN = D6
 *   - LED2_PIN = D7
 *
 * Arnés RJ45/UTP remoto (BNO085/LED — NO ETHERNET):
 *   1:+5V solo a VIN/5V del breakout Adafruit | 2:GND | 3:SDA | 4:LED1
 *   5:LED2 | 6:GND | 7:SCL | 8:GND
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

enum MovementState { MOVING = 0, AVERAGING = 1, LOCKED = 2 };
enum PPPState { SIN_PPP = 0, PPP_CONVERGING = 1, PPP_ESTABLE = 2 };
enum DynatestOutputMode { DYNATEST_OUTPUT_USB_CDC = 0, DYNATEST_OUTPUT_DISABLED = 1 };

constexpr unsigned long USB_BAUD = 115200;
constexpr unsigned long GNSS_BAUD = 115200;
constexpr DynatestOutputMode DYNATEST_OUTPUT_MODE = DYNATEST_OUTPUT_USB_CDC;
constexpr bool ENABLE_USB_DIAGNOSTICS = false;
static_assert(!(ENABLE_USB_DIAGNOSTICS && DYNATEST_OUTPUT_MODE == DYNATEST_OUTPUT_USB_CDC),
              "Serial/USB CDC no puede transportar logs y NMEA limpio simultaneamente.");

constexpr int LED1_PIN = 6;
constexpr int LED2_PIN = 7;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint8_t BNO085_I2C_ADDRESS = 0x4B;

constexpr double OFFSET_M = 0.55;
constexpr double DECLINATION_OFFSET_DEG = 1.0;
constexpr double EARTH_RADIUS_M = 6378137.0;
constexpr double SPEED_ENTER_STOP_MPS = 0.20;
constexpr double SPEED_EXIT_STOP_MPS = 0.30;
constexpr double NEW_LOCATION_DIST_M = 1.0;
constexpr double HDOP_FALLBACK = 1.0;

constexpr uint32_t STOP_CONFIRM_MS = 2000;
constexpr uint32_t SPEED_FRESHNESS_MS = 2000;
constexpr uint32_t GGA_FRESHNESS_MS = 2000;
constexpr uint32_t IMU_RETRY_MS = 5000;
constexpr uint32_t IMU_LOG_INTERVAL_MS = 5000;
constexpr uint32_t AVERAGING_WINDOW_MS = 15000;
constexpr uint32_t OUTPUT_PERIOD_MS = 100;
constexpr int MAX_SAMPLES = 160;

MovementState movementState = MOVING;
PPPState pppState = SIN_PPP;

uint32_t lastStopCheckMs = 0;
uint32_t averagingStartMs = 0;
uint32_t lastOutputMs = 0;
uint32_t lastSpeedUpdateMs = 0;
uint32_t lastGgaMs = 0;
uint32_t lastSampledGgaMs = 0;
uint32_t lastBnoMs = 0;
uint32_t lastImuRetryMs = 0;
uint32_t lastImuLogMs = 0;

bool gnssValid = false;
bool bnoAvailable = false;
bool lockedValid = false;
bool currentHdopValid = false;

double currentLat = 0.0;
double currentLon = 0.0;
double currentAlt = 0.0;
double currentSpeedMS = 0.0;
double currentCourseOverGround = 0.0;
double currentYaw = NAN;
int currentFixQ = 0;

double lockedRawLat = 0.0;
double lockedRawLon = 0.0;
double lockedAlt = 0.0;
double lockedYaw = NAN;
double lockedOutputLat = 0.0;
double lockedOutputLon = 0.0;

int satCount = 0;
char utcTime[16] = "000000.00";
char currentHdopField[16] = "1.0";

double latBuffer[MAX_SAMPLES];
double lonBuffer[MAX_SAMPLES];
double altBuffer[MAX_SAMPLES];
double yawBuffer[MAX_SAMPLES];
int sampleCount = 0;
int yawSampleCount = 0;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

static char gnssLineBuf[220];
static int gnssLineIdx = 0;

void diagLog(const char *format, ...) {
  if (!ENABLE_USB_DIAGNOSTICS) {
    return;
  }

  char buffer[192];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.println(buffer);
}

double toRadians(double deg) {
  return deg * PI / 180.0;
}

double toDegrees(double rad) {
  return rad * 180.0 / PI;
}

double normalizeAngle(double ang) {
  while (ang < 0.0) {
    ang += 360.0;
  }
  while (ang >= 360.0) {
    ang -= 360.0;
  }
  return ang;
}

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double dlat = toRadians(lat2 - lat1);
  const double dlon = toRadians(lon2 - lon1);
  const double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
                   cos(toRadians(lat1)) * cos(toRadians(lat2)) *
                       sin(dlon / 2.0) * sin(dlon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS_M * c;
}

double meanCircularDeg(double *buffer, int count) {
  if (count <= 0) {
    return NAN;
  }

  double sinSum = 0.0;
  double cosSum = 0.0;
  for (int i = 0; i < count; ++i) {
    const double radians = toRadians(buffer[i]);
    sinSum += sin(radians);
    cosSum += cos(radians);
  }

  return normalizeAngle(toDegrees(atan2(sinSum / count, cosSum / count)));
}

double trimmedMean(double *buffer, int count) {
  if (count <= 0) {
    return NAN;
  }

  static double sortBuffer[MAX_SAMPLES];
  memcpy(sortBuffer, buffer, count * sizeof(double));

  for (int i = 0; i < count - 1; ++i) {
    for (int j = 0; j < count - i - 1; ++j) {
      if (sortBuffer[j] > sortBuffer[j + 1]) {
        const double tmp = sortBuffer[j];
        sortBuffer[j] = sortBuffer[j + 1];
        sortBuffer[j + 1] = tmp;
      }
    }
  }

  const int trimCount = (int)ceil(count * 0.05);
  const int start = (trimCount * 2 >= count) ? 0 : trimCount;
  const int end = (trimCount * 2 >= count) ? count : count - trimCount;

  double sum = 0.0;
  int validCount = 0;
  for (int i = start; i < end; ++i) {
    if (!isnan(sortBuffer[i])) {
      sum += sortBuffer[i];
      ++validCount;
    }
  }

  return (validCount > 0) ? (sum / validCount) : NAN;
}

bool isGgaSentence(const char *line) {
  return line[0] == '$' && strlen(line) >= 6 &&
         line[3] == 'G' && line[4] == 'G' && line[5] == 'A';
}

bool isRmcSentence(const char *line) {
  return line[0] == '$' && strlen(line) >= 6 &&
         line[3] == 'R' && line[4] == 'M' && line[5] == 'C';
}

bool startsWithPppnava(const char *line) {
  return strncmp(line, "#PPPNAVA", 8) == 0;
}

bool verifyNmeaChecksum(const char *line) {
  const char *asterisk = strchr(line, '*');
  if (line[0] != '$' || asterisk == nullptr || strlen(asterisk) < 3) {
    return false;
  }

  unsigned char calculated = 0;
  for (const char *ptr = line + 1; ptr < asterisk; ++ptr) {
    calculated ^= (unsigned char)(*ptr);
  }

  unsigned int received = 0;
  if (sscanf(asterisk + 1, "%2x", &received) != 1) {
    return false;
  }

  return calculated == (unsigned char)received;
}

bool isFinitePositive(const char *field, double &value) {
  if (field == nullptr || field[0] == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  value = strtod(field, &endPtr);
  return endPtr != field && *endPtr == '\0' && isfinite(value) && value > 0.0;
}

void applyOffsetIfAvailable(double baseLat,
                            double baseLon,
                            double yawDeg,
                            double &outLat,
                            double &outLon) {
  outLat = baseLat;
  outLon = baseLon;

  if (isnan(yawDeg)) {
    return;
  }

  const double correctionBearing = normalizeAngle(yawDeg + 270.0);
  const double offsetRad = OFFSET_M / EARTH_RADIUS_M;
  const double bearingRad = toRadians(correctionBearing);
  const double latOffset = offsetRad * cos(bearingRad);
  const double lonOffset = offsetRad * sin(bearingRad) / cos(toRadians(baseLat));

  outLat += latOffset;
  outLon += lonOffset;
}

void formatNmeaCoordinate(double degreesDecimal,
                          bool latitude,
                          char *buffer,
                          size_t bufferSize,
                          char &hemisphere) {
  const double absoluteDegrees = fabs(degreesDecimal);
  const int wholeDegrees = (int)absoluteDegrees;
  const double minutes = (absoluteDegrees - wholeDegrees) * 60.0;

  hemisphere = latitude ? ((degreesDecimal >= 0.0) ? 'N' : 'S')
                        : ((degreesDecimal >= 0.0) ? 'E' : 'W');

  if (latitude) {
    snprintf(buffer, bufferSize, "%02d%07.4f", wholeDegrees, minutes);
  } else {
    snprintf(buffer, bufferSize, "%03d%07.4f", wholeDegrees, minutes);
  }
}

bool ggaIsFresh() {
  return gnssValid && (millis() - lastGgaMs) <= GGA_FRESHNESS_MS;
}

void parseGGA(const char *line) {
  if (!verifyNmeaChecksum(line)) {
    diagLog("[GNSS] GGA checksum error");
    return;
  }

  char lineCopy[220];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  char *asterisk = strchr(lineCopy, '*');
  if (asterisk == nullptr) {
    return;
  }
  *asterisk = '\0';

  char *fields[16] = {nullptr};
  int fieldCount = 0;
  fields[fieldCount++] = lineCopy;
  for (char *ptr = lineCopy; *ptr != '\0' && fieldCount < 16; ++ptr) {
    if (*ptr == ',') {
      *ptr = '\0';
      fields[fieldCount++] = ptr + 1;
    }
  }

  if (fieldCount < 10) {
    return;
  }

  const int fixQ = atoi(fields[6]);
  if (fixQ < 1) {
    gnssValid = false;
    return;
  }

  const char *latStr = fields[2];
  const char *latHem = fields[3];
  const char *lonStr = fields[4];
  const char *lonHem = fields[5];
  if (strlen(latStr) < 4 || strlen(lonStr) < 5) {
    return;
  }

  const int latDeg = (latStr[0] - '0') * 10 + (latStr[1] - '0');
  const double latMin = strtod(latStr + 2, nullptr);
  double lat = latDeg + latMin / 60.0;
  if (latHem[0] == 'S') {
    lat = -lat;
  }

  const int lonDeg = (lonStr[0] - '0') * 100 + (lonStr[1] - '0') * 10 + (lonStr[2] - '0');
  const double lonMin = strtod(lonStr + 3, nullptr);
  double lon = lonDeg + lonMin / 60.0;
  if (lonHem[0] == 'W') {
    lon = -lon;
  }

  const double alt = strtod(fields[9], nullptr);

  double hdop = 0.0;
  if (isFinitePositive(fields[8], hdop)) {
    currentHdopValid = true;
    strncpy(currentHdopField, fields[8], sizeof(currentHdopField) - 1);
    currentHdopField[sizeof(currentHdopField) - 1] = '\0';
  } else {
    currentHdopValid = false;
    snprintf(currentHdopField, sizeof(currentHdopField), "%.1f", HDOP_FALLBACK);
  }

  strncpy(utcTime, fields[1], sizeof(utcTime) - 1);
  utcTime[sizeof(utcTime) - 1] = '\0';
  satCount = atoi(fields[7]);
  currentFixQ = fixQ;
  currentLat = lat;
  currentLon = lon;
  currentAlt = alt;
  gnssValid = true;
  lastGgaMs = millis();

  diagLog("[GNSS] GGA valid");
}

void parseRMC(const char *line) {
  if (!verifyNmeaChecksum(line)) {
    return;
  }

  char lineCopy[220];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  char *asterisk = strchr(lineCopy, '*');
  if (asterisk == nullptr) {
    return;
  }
  *asterisk = '\0';

  char *fields[16] = {nullptr};
  int fieldCount = 0;
  fields[fieldCount++] = lineCopy;
  for (char *ptr = lineCopy; *ptr != '\0' && fieldCount < 16; ++ptr) {
    if (*ptr == ',') {
      *ptr = '\0';
      fields[fieldCount++] = ptr + 1;
    }
  }

  if (fieldCount < 9 || fields[2][0] != 'A') {
    return;
  }

  const double speedKnots = strtod(fields[7], nullptr);
  currentSpeedMS = speedKnots * 0.514444;
  currentCourseOverGround = strtod(fields[8], nullptr);
  lastSpeedUpdateMs = millis();
}

void parsePPPNAVA(const char *line) {
  if (!startsWithPppnava(line)) {
    return;
  }

  if (strstr(line, "PPP_CONVERGING") != nullptr) {
    pppState = PPP_CONVERGING;
    diagLog("[PPP] PPP_CONVERGING");
  } else if (strstr(line, "PPP_ESTABLE") != nullptr) {
    pppState = PPP_ESTABLE;
    diagLog("[PPP] PPP_ESTABLE");
  } else {
    pppState = SIN_PPP;
    diagLog("[PPP] PPPNAVA sin estado reconocido");
  }
}

void processGnssLine(const char *line) {
  if (isGgaSentence(line)) {
    parseGGA(line);
  } else if (isRmcSentence(line)) {
    parseRMC(line);
  } else if (startsWithPppnava(line)) {
    parsePPPNAVA(line);
  }
}

void readGNSSSerial() {
  while (Serial1.available() > 0) {
    const char c = (char)Serial1.read();

    if (c == '\r') {
      continue;
    }

    if (c == '$' || c == '#') {
      gnssLineIdx = 0;
      gnssLineBuf[gnssLineIdx++] = c;
      continue;
    }

    if (gnssLineIdx <= 0) {
      continue;
    }

    if (c == '\n') {
      gnssLineBuf[gnssLineIdx] = '\0';
      processGnssLine(gnssLineBuf);
      gnssLineIdx = 0;
    } else if (gnssLineIdx < (int)sizeof(gnssLineBuf) - 1) {
      gnssLineBuf[gnssLineIdx++] = c;
    } else {
      gnssLineIdx = 0;
    }
  }
}

bool initBNO085() {
  if (!bno08x.begin_I2C(BNO085_I2C_ADDRESS, &Wire)) {
    diagLog("[IMU] BNO085 init failed");
    return false;
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    diagLog("[IMU] Could not enable rotation vector");
    return false;
  }

  bnoAvailable = true;
  lastBnoMs = millis();
  diagLog("[IMU] BNO085 initialized");
  return true;
}

double readYaw() {
  const uint32_t nowMs = millis();

  if (!bnoAvailable) {
    if (nowMs - lastImuRetryMs >= IMU_RETRY_MS) {
      lastImuRetryMs = nowMs;
      bnoAvailable = initBNO085();
    }
    return NAN;
  }

  if (nowMs - lastBnoMs > IMU_RETRY_MS) {
    if (nowMs - lastImuLogMs >= IMU_LOG_INTERVAL_MS) {
      lastImuLogMs = nowMs;
      diagLog("[IMU] Timeout - yaw NAN");
    }
    currentYaw = NAN;
    return NAN;
  }

  if (bno08x.wasReset()) {
    diagLog("[IMU] Reset detected");
    bnoAvailable = initBNO085();
    if (!bnoAvailable) {
      return NAN;
    }
  }

  if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    const float i = sensorValue.un.rotationVector.i;
    const float j = sensorValue.un.rotationVector.j;
    const float k = sensorValue.un.rotationVector.k;
    const float real = sensorValue.un.rotationVector.real;

    const double yawRad = atan2(2.0 * (real * k + i * j),
                                1.0 - 2.0 * (j * j + k * k));
    const double yawMag = toDegrees(yawRad);
    currentYaw = normalizeAngle(yawMag - DECLINATION_OFFSET_DEG);
    lastBnoMs = nowMs;
  }

  return currentYaw;
}

int getOutputFixQ() {
  if (!ggaIsFresh()) {
    return 0;
  }

  int fixQ = currentFixQ;
  if (movementState == LOCKED && lockedValid && fixQ < 4) {
    fixQ = 4;
  }
  if (pppState == PPP_ESTABLE && fixQ < 4) {
    fixQ = 4;
  }
  if (pppState == PPP_CONVERGING && fixQ < 2) {
    fixQ = 2;
  }

  return (fixQ >= 1) ? fixQ : 0;
}

void resetAveragingBuffers() {
  sampleCount = 0;
  yawSampleCount = 0;
  memset(latBuffer, 0, sizeof(latBuffer));
  memset(lonBuffer, 0, sizeof(lonBuffer));
  memset(altBuffer, 0, sizeof(altBuffer));
  memset(yawBuffer, 0, sizeof(yawBuffer));
  lastSampledGgaMs = 0;
}

void updateMovementState() {
  const uint32_t nowMs = millis();
  const bool speedFresh = (nowMs - lastSpeedUpdateMs) <= SPEED_FRESHNESS_MS;
  const bool ggaFresh = ggaIsFresh();

  if (!speedFresh) {
    currentSpeedMS = 0.0;
  }
  if (!ggaFresh) {
    gnssValid = false;
  }

  switch (movementState) {
    case MOVING:
      if (speedFresh && currentSpeedMS < SPEED_ENTER_STOP_MPS) {
        if (lastStopCheckMs == 0) {
          lastStopCheckMs = nowMs;
        }
      } else if (speedFresh && currentSpeedMS >= SPEED_EXIT_STOP_MPS) {
        lastStopCheckMs = 0;
      }

      if (lastStopCheckMs > 0 && (nowMs - lastStopCheckMs) >= STOP_CONFIRM_MS) {
        averagingStartMs = nowMs;
        resetAveragingBuffers();
        movementState = AVERAGING;
        lockedValid = false;
        diagLog("[STATE] MOVING -> AVERAGING");
      }
      break;

    case AVERAGING:
      if (lastGgaMs != 0 && lastGgaMs != lastSampledGgaMs && sampleCount < MAX_SAMPLES && ggaFresh) {
        latBuffer[sampleCount] = currentLat;
        lonBuffer[sampleCount] = currentLon;
        altBuffer[sampleCount] = currentAlt;
        ++sampleCount;
        lastSampledGgaMs = lastGgaMs;

        if (!isnan(currentYaw) && yawSampleCount < MAX_SAMPLES) {
          yawBuffer[yawSampleCount++] = currentYaw;
        }
      }

      if (speedFresh && currentSpeedMS > SPEED_EXIT_STOP_MPS) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        resetAveragingBuffers();
        diagLog("[STATE] AVERAGING -> MOVING (speed)");
        break;
      }

      if ((nowMs - averagingStartMs) >= AVERAGING_WINDOW_MS && sampleCount > 0) {
        lockedRawLat = trimmedMean(latBuffer, sampleCount);
        lockedRawLon = trimmedMean(lonBuffer, sampleCount);
        lockedAlt = trimmedMean(altBuffer, sampleCount);
        lockedYaw = meanCircularDeg(yawBuffer, yawSampleCount);

        if (isnan(lockedRawLat) || isnan(lockedRawLon)) {
          movementState = MOVING;
          lockedValid = false;
          resetAveragingBuffers();
          diagLog("[STATE] AVERAGING failed");
          break;
        }

        applyOffsetIfAvailable(lockedRawLat, lockedRawLon, lockedYaw, lockedOutputLat, lockedOutputLon);
        movementState = LOCKED;
        lockedValid = true;
        lastStopCheckMs = 0;
        diagLog("[STATE] AVERAGING -> LOCKED");
      }
      break;

    case LOCKED:
      if (speedFresh && currentSpeedMS > SPEED_EXIT_STOP_MPS) {
        movementState = MOVING;
        lockedValid = false;
        lastStopCheckMs = 0;
        resetAveragingBuffers();
        diagLog("[STATE] LOCKED -> MOVING (speed)");
        break;
      }

      if (ggaFresh) {
        const double distanceMoved = haversineMeters(currentLat, currentLon, lockedRawLat, lockedRawLon);
        if (distanceMoved > NEW_LOCATION_DIST_M) {
          movementState = MOVING;
          lockedValid = false;
          lastStopCheckMs = 0;
          resetAveragingBuffers();
          diagLog("[STATE] LOCKED -> MOVING (distance)");
        }
      }
      break;
  }
}

void updateLeds() {
  const uint32_t nowMs = millis();
  static uint32_t startupMs = 0;
  if (startupMs == 0) {
    startupMs = nowMs;
  }

  if (nowMs - startupMs < 1200) {
    const bool on = ((nowMs - startupMs) % 400) < 200;
    digitalWrite(LED1_PIN, on ? HIGH : LOW);
    digitalWrite(LED2_PIN, on ? HIGH : LOW);
    return;
  }

  if (!ggaIsFresh()) {
    digitalWrite(LED1_PIN, LOW);
  } else if (pppState == PPP_CONVERGING) {
    digitalWrite(LED1_PIN, (nowMs % 400) < 200 ? HIGH : LOW);
  } else {
    digitalWrite(LED1_PIN, HIGH);
  }

  if (movementState == MOVING) {
    digitalWrite(LED2_PIN, LOW);
  } else if (movementState == AVERAGING) {
    digitalWrite(LED2_PIN, (nowMs % 1000) < 500 ? HIGH : LOW);
  } else if (movementState == LOCKED && lockedValid) {
    digitalWrite(LED2_PIN, HIGH);
  } else {
    digitalWrite(LED2_PIN, LOW);
  }
}

void sendDynatestSentence(const char *sentence) {
  if (DYNATEST_OUTPUT_MODE == DYNATEST_OUTPUT_USB_CDC) {
    Serial.write((const uint8_t *)sentence, strlen(sentence));
  }
}

void transmitDynatestOutput() {
  const uint32_t nowMs = millis();
  if (nowMs - lastOutputMs < OUTPUT_PERIOD_MS) {
    return;
  }
  lastOutputMs = nowMs;

  if (!ggaIsFresh()) {
    return;
  }

  const int fixQ = getOutputFixQ();
  if (fixQ < 1) {
    return;
  }

  double outputLat = currentLat;
  double outputLon = currentLon;
  double outputAlt = currentAlt;

  if (movementState == LOCKED && lockedValid) {
    outputLat = lockedOutputLat;
    outputLon = lockedOutputLon;
    outputAlt = lockedAlt;
  } else {
    applyOffsetIfAvailable(currentLat, currentLon, currentYaw, outputLat, outputLon);
  }

  char latField[16];
  char lonField[16];
  char latHem = 'N';
  char lonHem = 'E';
  formatNmeaCoordinate(outputLat, true, latField, sizeof(latField), latHem);
  formatNmeaCoordinate(outputLon, false, lonField, sizeof(lonField), lonHem);

  char sentence[160];
  snprintf(sentence,
           sizeof(sentence),
           "$GCGGA,%s,%s,%c,%s,%c,%d,%02d,%s,%.1f,M,0.0,M,,",
           utcTime,
           latField,
           latHem,
           lonField,
           lonHem,
           fixQ,
           satCount,
           currentHdopValid ? currentHdopField : "1.0",
           outputAlt);

  unsigned char checksum = 0;
  for (const char *ptr = sentence + 1; *ptr != '\0'; ++ptr) {
    checksum ^= (unsigned char)(*ptr);
  }

  char output[176];
  snprintf(output, sizeof(output), "%s*%02X\r\n", sentence, checksum);
  sendDynatestSentence(output);
}

void setup() {
  Serial.begin(USB_BAUD);
  Serial1.begin(GNSS_BAUD);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  initBNO085();

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);

  diagLog("[SETUP] UNO R4 WiFi Rev.1 ready");
}

void loop() {
  readGNSSSerial();
  readYaw();
  updateMovementState();
  updateLeds();
  transmitDynatestOutput();
}
