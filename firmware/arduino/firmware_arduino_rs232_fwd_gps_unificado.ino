#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

// ======================================================
//  RS232 FWD GPS - UNIFICADO (.ino)
//  - Lee GNSS UM980 por UART_IN
//  - Lee heading de magnetómetro I2C (QMC5883L)
//  - Calcula coordenada corregida por offset (punto referencia Dynatest)
//  - Genera y ENVÍA $GPGGA por UART_OUT -> MAX3232
//  - En detenido: promedio circular de 15s y salida a 10Hz
//  - Diseño no bloqueante para operación prolongada
// ======================================================

// ================== CONFIGURACIÓN =====================
// --- UART entrada GNSS (UM980 TX3/RX3 hacia ESP32) ---
static const int GNSS_RX_PIN = 44;      // ESP32 RX <- TX3 UM980
static const int GNSS_TX_PIN = 43;      // ESP32 TX -> RX3 UM980 (opcional)
static const uint32_t GNSS_BAUD = 115200;

// --- UART salida RS232 (ESP32 -> MAX3232 / Dynatest Embedded) ---
static const int OUT_TX_PIN = 17;       // ESP32 TX -> RX MAX3232
static const int OUT_RX_PIN = 18;       // ESP32 RX <- TX MAX3232 (opcional)
static const uint32_t OUT_BAUD = 38400; // Dynatest Embedded esperado 38400

// --- I2C magnetómetro (vía RJ45 en instalación final) ---
static const int I2C_SDA_PIN = 8;
static const int I2C_SCL_PIN = 9;
static const uint8_t MAG_ADDR = 0x0D;   // QMC5883L típico

// --- LEDs (vía RJ45 según arnés externo) ---
static const int LED_GNSS = 2;          // fix
static const int LED_MAG = 4;           // pulso lectura mag
static const int LED_ERR = 5;           // estado error

// --- Navegación/offset ---
// offsetAlongHeadingMeters:
//   >0 : punto referencia delante de la antena
//   <0 : punto referencia detrás de la antena
// Para mástil donde antena+magnetómetro van delante del centro del plato,
// usar valor negativo para aplicar heading + 180° internamente.
static double offsetAlongHeadingMeters = -1.50;
// offsetLateralMeters:
//   >0 : derecha del avance
//   <0 : izquierda del avance
static double offsetLateralMeters = 0.0;

static double declinationDeg = -4.8;    // declinación local (ajustar campo)
static float headingAlpha = 0.20f;      // filtro heading 0..1

// --- Política fix quality ---
// NMEA no define un valor universal propio para HAS. Por defecto se conserva
// el fix quality recibido del UM980 sin remapeo.
static const bool FIXQ_REMAP_ENABLED = false;

// --- Detección detenido y promedio ---
static const double STOP_SPEED_MS_ENTER = 0.20;  // entra a detenido
static const double STOP_SPEED_MS_EXIT = 0.30;   // sale de detenido (histéresis)
static const uint32_t STOP_CONFIRM_MS = 2000;    // confirmar 2s
static const uint32_t AVG_WINDOW_MS = 15000;     // promedio 15s
static const uint32_t OUT_PERIOD_MS = 100;       // 10Hz
static const uint32_t GGA_FRESH_MAX_MS = 2000;

// --- Fallback heading por COG ---
static const double COG_HEADING_MIN_SPEED_MS = 0.80;
static const uint32_t COG_MAX_AGE_MS = 2000;
static const uint32_t HEADING_FALLBACK_MAX_AGE_MS = 5000;
static const uint32_t SPEED_FRESH_MAX_MS = 2000;

// --- Robustez NMEA ---
static const size_t NMEA_LINE_MAX = 224;

// --- Robustez magnetómetro ---
static const uint8_t MAG_INIT_RETRIES = 3;
static const uint8_t MAG_FAIL_STREAK_LIMIT = 5;
static const uint32_t MAG_RECOVERY_RETRY_MS = 1000;
static const uint32_t MAG_HEADING_MAX_AGE_MS = 500;

// --- LEDs no bloqueantes ---
static const uint32_t LED_MAG_PULSE_MS = 10;
static const uint32_t LED_ERR_BLINK_MS = 250;

// ================== OBJETOS ===========================
HardwareSerial GNSS(1);  // entrada UM980
HardwareSerial OUT(2);   // salida al MAX3232

// Estado GNSS
bool gnssFix = false;
double gnssLat = NAN, gnssLon = NAN, gnssAlt = NAN;
int gnssFixQ = 0;
uint8_t gnssSats = 0;
double gnssSpeedMS = NAN;
bool gnssSpeedValid = false;
uint32_t gnssSpeedMsTimestamp = 0;

char gnssUtcRaw[16] = {0};
uint32_t gnssUtcCentis = 0;
bool gnssUtcParsed = false;
uint32_t gnssUtcAdvanceMs = 0;

// Estado heading/fuentes
bool magOk = false;
bool magError = false;
float magHeading = NAN;
float headingFiltered = NAN;

double lastHeadingTrue = NAN;
bool hasLastHeadingTrue = false;
uint32_t lastHeadingTrueMs = 0;

double lastCogDeg = NAN;
bool hasLastCog = false;
uint32_t lastCogMs = 0;

// Tiempos/contadores
uint32_t lastLogMs = 0;
uint32_t ggaCount = 0;
uint32_t outCount = 0;
uint32_t lastOutMs = 0;
uint32_t malformedNmeaCount = 0;
uint32_t lastGgaMs = 0;

// Estado detenido/movimiento
bool isStopped = false;
uint32_t stopCandidateSince = 0;
uint32_t moveCandidateSince = 0;

// Última coordenada para salida instantánea
bool hasOutputNow = false;
double outputNowLat = NAN, outputNowLon = NAN;

// Buffer promedio 15s (anillo)
struct Sample {
  double lat;
  double lon;
  uint32_t ms;
};
static const int MAX_SAMPLES = 220; // suficiente para >15s a 10Hz
Sample avgBuf[MAX_SAMPLES];
int avgHead = 0;   // próximo write
int avgCount = 0;  // elementos válidos

bool avgHas = false;
double avgLat = NAN, avgLon = NAN;

// Lector robusto de líneas NMEA
char nmeaLineBuf[NMEA_LINE_MAX];
size_t nmeaLineLen = 0;
bool nmeaOverflow = false;

// LED no bloqueante
uint32_t ledMagPulseUntil = 0;
uint32_t ledErrBlinkToggleMs = 0;
bool ledErrState = false;

// Magnetómetro recuperación
uint8_t magFailStreak = 0;
uint32_t lastMagRecoveryTryMs = 0;

// Autotest no bloqueante
struct AutoTestState {
  bool active;
  uint32_t startMs;
  uint32_t ggaIn;
  uint32_t ggaOut;
  uint32_t magReads;
  bool magAnyOk;
  bool gnssAnyFix;
};
AutoTestState autoTest = {false, 0, 0, 0, 0, false, false};

// ================== UTILIDADES ========================
static const double R_EARTH = 6378137.0;

double wrap360(double v) {
  while (v < 0.0) v += 360.0;
  while (v >= 360.0) v -= 360.0;
  return v;
}

bool parseDoubleStrict(const char* s, double& out) {
  if (!s || s[0] == '\0') return false;

  size_t i = 0;
  bool neg = false;
  if (s[i] == '+' || s[i] == '-') {
    neg = (s[i] == '-');
    i++;
  }

  bool anyDigit = false;
  double intPart = 0.0;
  while (s[i] >= '0' && s[i] <= '9') {
    anyDigit = true;
    intPart = intPart * 10.0 + static_cast<double>(s[i] - '0');
    i++;
  }

  double fracPart = 0.0;
  double fracDiv = 1.0;
  if (s[i] == '.') {
    i++;
    while (s[i] >= '0' && s[i] <= '9') {
      anyDigit = true;
      fracPart = fracPart * 10.0 + static_cast<double>(s[i] - '0');
      fracDiv *= 10.0;
      i++;
    }
  }

  if (!anyDigit || s[i] != '\0') return false;

  double value = intPart + (fracPart / fracDiv);
  if (neg) value = -value;
  out = value;
  return true;
}

bool parseUIntStrict(const char* s, uint32_t& out) {
  if (!s || s[0] == '\0') return false;
  uint32_t value = 0;
  for (size_t i = 0; s[i] != '\0'; ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    const uint32_t digit = static_cast<uint32_t>(s[i] - '0');
    if (value > (0xFFFFFFFFu - digit) / 10u) return false;
    value = value * 10u + digit;
  }
  out = value;
  return true;
}

bool isSentenceType(const char* line, const char* type3) {
  return line &&
         strlen(line) >= 6 &&
         line[0] == '$' &&
         line[3] == type3[0] &&
         line[4] == type3[1] &&
         line[5] == type3[2];
}

uint8_t nmeaChecksum(const char* sentenceNoDollarNoStar) {
  uint8_t cs = 0;
  if (!sentenceNoDollarNoStar) return cs;
  for (size_t i = 0; sentenceNoDollarNoStar[i] != '\0'; ++i) {
    cs ^= static_cast<uint8_t>(sentenceNoDollarNoStar[i]);
  }
  return cs;
}

int hexDigitToInt(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

bool verifyNmeaChecksum(const char* line) {
  if (!line || line[0] != '$') return false;
  const char* star = strchr(line, '*');
  if (!star) return false;
  if (star[1] == '\0' || star[2] == '\0') return false;

  const int high = hexDigitToInt(star[1]);
  const int low = hexDigitToInt(star[2]);
  if (high < 0 || low < 0) return false;

  uint8_t calc = 0;
  for (const char* p = line + 1; p < star; ++p) calc ^= static_cast<uint8_t>(*p);
  const uint8_t expected = static_cast<uint8_t>((high << 4) | low);
  return calc == expected;
}

bool nmeaToDecimalDegrees(const char* v, const char* hemi, bool isLat, double& outDeg) {
  if (!v || !hemi || hemi[0] == '\0') return false;

  double raw = 0.0;
  if (!parseDoubleStrict(v, raw)) return false;

  const int deg = static_cast<int>(raw / 100.0);
  const double minutes = raw - (static_cast<double>(deg) * 100.0);
  if (minutes < 0.0 || minutes >= 60.0) return false;

  double dec = static_cast<double>(deg) + (minutes / 60.0);

  const char h = static_cast<char>(toupper(static_cast<unsigned char>(hemi[0])));
  if (isLat) {
    if (h == 'S') dec = -dec;
    else if (h != 'N') return false;
    if (dec < -90.0 || dec > 90.0) return false;
  } else {
    if (h == 'W') dec = -dec;
    else if (h != 'E') return false;
    if (dec < -180.0 || dec > 180.0) return false;
  }

  outDeg = dec;
  return true;
}

void decimalDegreesToNmea(double deg, bool isLat, char* valueOut, size_t valueOutSize, char& hemiOut) {
  const double a = fabs(deg);
  const int d = static_cast<int>(a);
  const double m = (a - static_cast<double>(d)) * 60.0;

  if (isLat) {
    snprintf(valueOut, valueOutSize, "%02d%07.4f", d, m); // ddmm.mmmm
    hemiOut = (deg >= 0.0) ? 'N' : 'S';
  } else {
    snprintf(valueOut, valueOutSize, "%03d%07.4f", d, m); // dddmm.mmmm
    hemiOut = (deg >= 0.0) ? 'E' : 'W';
  }
}

void destinationPoint(double latDeg, double lonDeg, double bearingDeg, double distM,
                      double& outLatDeg, double& outLonDeg) {
  const double lat1 = latDeg * DEG_TO_RAD;
  const double lon1 = lonDeg * DEG_TO_RAD;
  const double brng = bearingDeg * DEG_TO_RAD;
  const double ang = distM / R_EARTH;

  const double sinLat1 = sin(lat1);
  const double cosLat1 = cos(lat1);
  const double sinAng = sin(ang);
  const double cosAng = cos(ang);

  const double sinLat2 = sinLat1 * cosAng + cosLat1 * sinAng * cos(brng);
  const double lat2 = asin(sinLat2);

  const double y = sin(brng) * sinAng * cosLat1;
  const double x = cosAng - sinLat1 * sinLat2;
  const double lon2 = lon1 + atan2(y, x);

  outLatDeg = lat2 * RAD_TO_DEG;
  outLonDeg = lon2 * RAD_TO_DEG;
}

int splitCsvInPlace(char* s, char* out[], int maxFields) {
  if (!s || !out || maxFields <= 0) return 0;
  int c = 0;
  out[c++] = s;
  for (size_t i = 0; s[i] != '\0' && c < maxFields; ++i) {
    if (s[i] == ',') {
      s[i] = '\0';
      out[c++] = &s[i + 1];
    }
  }
  return c;
}

bool parseUtcToCentis(const char* utc, uint32_t& outCentis) {
  if (!utc) return false;
  const size_t len = strlen(utc);
  if (len < 6) return false;

  char hhStr[3] = {utc[0], utc[1], '\0'};
  char mmStr[3] = {utc[2], utc[3], '\0'};
  char ssStr[3] = {utc[4], utc[5], '\0'};

  uint32_t hh = 0, mm = 0, ss = 0;
  if (!parseUIntStrict(hhStr, hh) || !parseUIntStrict(mmStr, mm) || !parseUIntStrict(ssStr, ss)) return false;
  if (hh > 23 || mm > 59 || ss > 59) return false;

  uint32_t centis = (hh * 360000u) + (mm * 6000u) + (ss * 100u);

  if (len > 6 && utc[6] == '.') {
    uint32_t frac = 0;
    uint32_t mult = 10;
    uint8_t fracDigits = 0;
    for (size_t i = 7; utc[i] != '\0'; ++i) {
      if (utc[i] < '0' || utc[i] > '9') return false;
      if (fracDigits < 2) {
        frac += static_cast<uint32_t>(utc[i] - '0') * mult;
        mult /= 10;
      }
      fracDigits++;
    }
    if (fracDigits == 0) return false;
    centis += frac;
  }

  outCentis = centis;
  return true;
}

void formatUtcFromCentis(uint32_t centis, char* out, size_t outSize) {
  static const uint32_t DAY_CENTIS = 24u * 3600u * 100u;
  centis %= DAY_CENTIS;

  const uint32_t hh = centis / 360000u;
  centis %= 360000u;
  const uint32_t mm = centis / 6000u;
  centis %= 6000u;
  const uint32_t ss = centis / 100u;
  const uint32_t cs = centis % 100u;

  snprintf(out, outSize, "%02lu%02lu%02lu.%02lu",
           static_cast<unsigned long>(hh),
           static_cast<unsigned long>(mm),
           static_cast<unsigned long>(ss),
           static_cast<unsigned long>(cs));
}

int mapFixQuality(int rawFixQ) {
  if (!FIXQ_REMAP_ENABLED) return rawFixQ;
  // Política opcional/experimental, dejar explícita si se activa.
  return rawFixQ;
}

bool buildCorrectedGPGGA(char* outSentence, size_t outSize,
                         const char* utc, double lat, double lon,
                         int fixQ, uint8_t sats, double altM) {
  if (!outSentence || outSize < 16 || !utc) return false;

  char latStr[16];
  char lonStr[16];
  char ns = 'N';
  char ew = 'E';

  decimalDegreesToNmea(lat, true, latStr, sizeof(latStr), ns);
  decimalDegreesToNmea(lon, false, lonStr, sizeof(lonStr), ew);

  char body[192];
  const int written = snprintf(body, sizeof(body),
                               "GPGGA,%s,%s,%c,%s,%c,%d,%u,1.0,%.2f,M,0.0,M,,",
                               utc,
                               latStr,
                               ns,
                               lonStr,
                               ew,
                               mapFixQuality(fixQ),
                               static_cast<unsigned int>(sats),
                               altM);
  if (written <= 0 || static_cast<size_t>(written) >= sizeof(body)) return false;

  const uint8_t cs = nmeaChecksum(body);
  const int outWritten = snprintf(outSentence, outSize, "$%s*%02X", body, cs);
  return (outWritten > 0 && static_cast<size_t>(outWritten) < outSize);
}

void avgClear() {
  avgHead = 0;
  avgCount = 0;
  avgHas = false;
  avgLat = NAN;
  avgLon = NAN;
}

void avgAdd(double lat, double lon, uint32_t nowMs) {
  avgBuf[avgHead].lat = lat;
  avgBuf[avgHead].lon = lon;
  avgBuf[avgHead].ms = nowMs;
  avgHead = (avgHead + 1) % MAX_SAMPLES;
  if (avgCount < MAX_SAMPLES) avgCount++;
}

void avgCompute15s(uint32_t nowMs) {
  double sumLat = 0.0;
  double sumLon = 0.0;
  int cnt = 0;

  for (int i = 0; i < avgCount; ++i) {
    int idx = avgHead - 1 - i;
    if (idx < 0) idx += MAX_SAMPLES;

    const uint32_t age = nowMs - avgBuf[idx].ms;
    if (age <= AVG_WINDOW_MS) {
      sumLat += avgBuf[idx].lat;
      sumLon += avgBuf[idx].lon;
      cnt++;
    } else {
      break;
    }
  }

  if (cnt > 0) {
    avgLat = sumLat / static_cast<double>(cnt);
    avgLon = sumLon / static_cast<double>(cnt);
    avgHas = true;
  } else {
    avgHas = false;
  }
}

// ================== MAGNETÓMETRO ======================
bool magWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool magReadBytes(uint8_t reg, uint8_t* data, uint8_t len) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t got = Wire.requestFrom(static_cast<int>(MAG_ADDR), static_cast<int>(len));
  if (got != len) return false;

  for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

bool initMagnetometerQMCOnce() {
  if (!magWrite8(0x0B, 0x01)) return false; // set/reset period
  if (!magWrite8(0x09, 0x1D)) return false; // OSR=512 RNG=8G ODR=200Hz CONT
  return true;
}

bool initMagnetometerQMC() {
  for (uint8_t i = 0; i < MAG_INIT_RETRIES; ++i) {
    if (initMagnetometerQMCOnce()) return true;
  }
  return false;
}

bool readMagHeading(float& headingDegOut) {
  uint8_t raw[6];
  if (!magReadBytes(0x00, raw, 6)) return false;

  const int16_t x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
  const int16_t y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
  if (x == 0 && y == 0) return false;

  float hdg = atan2f(static_cast<float>(y), static_cast<float>(x)) * 180.0f / PI;
  if (hdg < 0.0f) hdg += 360.0f;
  headingDegOut = hdg;
  return true;
}

void ensureMagRecovery(uint32_t nowMs) {
  if (magOk) return;
  if (nowMs - lastMagRecoveryTryMs < MAG_RECOVERY_RETRY_MS) return;

  lastMagRecoveryTryMs = nowMs;
  const bool recovered = initMagnetometerQMC();
  magOk = recovered;

  if (recovered) {
    magFailStreak = 0;
    magError = false;
    Serial.println("[mag] Recuperado tras reintento I2C");
  } else {
    magError = true;
  }
}

void updateHeadingFromMag(uint32_t nowMs) {
  (void)nowMs;
  if (!magOk) return;

  float h = NAN;
  if (readMagHeading(h)) {
    magHeading = h;
    autoTest.magReads++;
    autoTest.magAnyOk = true;

    if (isnan(headingFiltered)) {
      headingFiltered = h;
    } else {
      const float a = headingFiltered * DEG_TO_RAD;
      const float b = h * DEG_TO_RAD;
      const float sx = (1.0f - headingAlpha) * cosf(a) + headingAlpha * cosf(b);
      const float sy = (1.0f - headingAlpha) * sinf(a) + headingAlpha * sinf(b);
      headingFiltered = atan2f(sy, sx) * 180.0f / PI;
      if (headingFiltered < 0.0f) headingFiltered += 360.0f;
    }

    lastHeadingTrue = wrap360(static_cast<double>(headingFiltered) + declinationDeg);
    hasLastHeadingTrue = true;
    lastHeadingTrueMs = nowMs;
    magFailStreak = 0;
    magError = false;
    ledMagPulseUntil = millis() + LED_MAG_PULSE_MS;
    digitalWrite(LED_MAG, HIGH);
    return;
  }

  if (magFailStreak < 255) magFailStreak++;
  if (magFailStreak >= MAG_FAIL_STREAK_LIMIT) {
    magOk = false;
    magError = true;
    Serial.println("[mag] Error de lectura I2C, entrando en modo recuperación");
  }
}

// ================== LEDS ==============================
void setupLeds() {
  pinMode(LED_GNSS, OUTPUT);
  pinMode(LED_MAG, OUTPUT);
  pinMode(LED_ERR, OUTPUT);
  digitalWrite(LED_GNSS, LOW);
  digitalWrite(LED_MAG, LOW);
  digitalWrite(LED_ERR, LOW);
}

bool gnssFixIsFresh(uint32_t nowMs);

void updateLeds(uint32_t nowMs) {
  const bool freshFix = gnssFixIsFresh(nowMs);
  digitalWrite(LED_GNSS, freshFix ? HIGH : LOW);

  if (ledMagPulseUntil != 0 && static_cast<int32_t>(nowMs - ledMagPulseUntil) >= 0) {
    digitalWrite(LED_MAG, LOW);
    ledMagPulseUntil = 0;
  }

  const bool errorActive = (!freshFix) || magError;
  if (!errorActive) {
    digitalWrite(LED_ERR, LOW);
    ledErrState = false;
    ledErrBlinkToggleMs = nowMs;
    return;
  }

  if (nowMs - ledErrBlinkToggleMs >= LED_ERR_BLINK_MS) {
    ledErrBlinkToggleMs = nowMs;
    ledErrState = !ledErrState;
    digitalWrite(LED_ERR, ledErrState ? HIGH : LOW);
  }
}

// ================== PARSERS NMEA ======================
struct GgaData {
  bool valid;
  double lat;
  double lon;
  double alt;
  int fixQ;
  uint8_t sats;
  char utc[16];
};

bool parseGGA(const char* line, GgaData& out) {
  out.valid = false;
  if (!isSentenceType(line, "GGA")) return false;

  char work[NMEA_LINE_MAX];
  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';

  char* star = strchr(work, '*');
  if (star) *star = '\0';

  char* fields[20];
  const int n = splitCsvInPlace(work, fields, 20);
  if (n < 10) return false;

  double lat = NAN;
  double lon = NAN;
  if (!nmeaToDecimalDegrees(fields[2], fields[3], true, lat)) return false;
  if (!nmeaToDecimalDegrees(fields[4], fields[5], false, lon)) return false;

  uint32_t fixQu = 0;
  uint32_t satsu = 0;
  double alt = 0.0;

  if (!parseUIntStrict(fields[6], fixQu)) fixQu = 0;
  if (!parseUIntStrict(fields[7], satsu)) satsu = 0;
  if (!parseDoubleStrict(fields[9], alt)) alt = isfinite(gnssAlt) ? gnssAlt : 0.0;

  strncpy(out.utc, fields[1] ? fields[1] : "", sizeof(out.utc) - 1);
  out.utc[sizeof(out.utc) - 1] = '\0';
  out.lat = lat;
  out.lon = lon;
  out.alt = alt;
  out.fixQ = static_cast<int>(fixQu);
  out.sats = static_cast<uint8_t>(satsu > 255u ? 255u : satsu);
  out.valid = true;
  return true;
}

bool parseVTG(const char* line, bool& speedValid, double& speedMs, bool& cogValid, double& cogDeg) {
  speedValid = false;
  cogValid = false;
  if (!isSentenceType(line, "VTG")) return false;

  char work[NMEA_LINE_MAX];
  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';

  char* star = strchr(work, '*');
  if (star) *star = '\0';

  char* fields[20];
  const int n = splitCsvInPlace(work, fields, 20);
  if (n < 9) return true;

  double kmh = 0.0;
  if (parseDoubleStrict(fields[7], kmh)) {
    speedMs = kmh / 3.6;
    speedValid = true;
  }

  const char* trueDesignator = (fields[2] && fields[2][0] != '\0') ? fields[2] : nullptr;
  const char* trueCourseField = fields[1];
  if (!(trueDesignator && trueDesignator[0] == 'T' && trueDesignator[1] == '\0') && n > 3) {
    trueDesignator = fields[3];
    trueCourseField = fields[2];
  }

  double cog = 0.0;
  if (trueDesignator && trueDesignator[0] == 'T' && trueDesignator[1] == '\0' &&
      parseDoubleStrict(trueCourseField, cog)) {
    cogDeg = wrap360(cog);
    cogValid = true;
  }

  return true;
}

bool parseRMC(const char* line, bool& speedValid, double& speedMs, bool& cogValid, double& cogDeg) {
  speedValid = false;
  cogValid = false;
  if (!isSentenceType(line, "RMC")) return false;

  char work[NMEA_LINE_MAX];
  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';

  char* star = strchr(work, '*');
  if (star) *star = '\0';

  char* fields[20];
  const int n = splitCsvInPlace(work, fields, 20);
  if (n < 9) return true;
  if (!(fields[2] && fields[2][0] == 'A' && fields[2][1] == '\0')) return true;

  double knots = 0.0;
  if (parseDoubleStrict(fields[7], knots)) {
    speedMs = knots * 0.514444;
    speedValid = true;
  }

  double cog = 0.0;
  if (parseDoubleStrict(fields[8], cog)) {
    cogDeg = wrap360(cog);
    cogValid = true;
  }

  return true;
}

bool nextNmeaLineFromGnss(char* outLine, size_t outSize) {
  if (!outLine || outSize < 2) return false;

  while (GNSS.available()) {
    const char c = static_cast<char>(GNSS.read());

    if (c == '\r') continue;

    if (c == '\n') {
      if (nmeaOverflow) {
        nmeaOverflow = false;
        nmeaLineLen = 0;
        malformedNmeaCount++;
        continue;
      }

      if (nmeaLineLen == 0) continue;

      nmeaLineBuf[nmeaLineLen] = '\0';
      strncpy(outLine, nmeaLineBuf, outSize - 1);
      outLine[outSize - 1] = '\0';
      nmeaLineLen = 0;
      return true;
    }

    if (nmeaOverflow) continue;

    if (nmeaLineLen < (NMEA_LINE_MAX - 1)) {
      nmeaLineBuf[nmeaLineLen++] = c;
    } else {
      nmeaOverflow = true;
    }
  }

  return false;
}

bool resolveHeadingTrue(uint32_t nowMs, double& headingTrueOut) {
  if (magOk && hasLastHeadingTrue && isfinite(lastHeadingTrue) &&
      (nowMs - lastHeadingTrueMs <= MAG_HEADING_MAX_AGE_MS)) {
    headingTrueOut = wrap360(lastHeadingTrue);
    return true;
  }

  if (gnssSpeedValid && gnssSpeedMS > COG_HEADING_MIN_SPEED_MS && hasLastCog) {
    if (nowMs - lastCogMs <= COG_MAX_AGE_MS) {
      headingTrueOut = wrap360(lastCogDeg);
      lastHeadingTrue = headingTrueOut;
      hasLastHeadingTrue = true;
      lastHeadingTrueMs = nowMs;
      return true;
    }
  }

  if (hasLastHeadingTrue && isfinite(lastHeadingTrue) &&
      (nowMs - lastHeadingTrueMs <= HEADING_FALLBACK_MAX_AGE_MS)) {
    headingTrueOut = wrap360(lastHeadingTrue);
    return true;
  }

  return false;
}

void applyOffsetFromAntennaToReference(double inLat, double inLon,
                                       bool headingAvailable, double headingTrueDeg,
                                       double& outLat, double& outLon) {
  outLat = inLat;
  outLon = inLon;

  if (!headingAvailable) {
    return;
  }

  if (fabs(offsetAlongHeadingMeters) > 0.0001) {
    double bearing = headingTrueDeg;
    double dist = offsetAlongHeadingMeters;
    if (dist < 0.0) {
      bearing = wrap360(bearing + 180.0);
      dist = -dist;
    }
    destinationPoint(outLat, outLon, bearing, dist, outLat, outLon);
  }

  if (fabs(offsetLateralMeters) > 0.0001) {
    const double bearing = wrap360(headingTrueDeg + (offsetLateralMeters >= 0.0 ? 90.0 : -90.0));
    const double dist = fabs(offsetLateralMeters);
    destinationPoint(outLat, outLon, bearing, dist, outLat, outLon);
  }
}

bool computeUtcForOutput(uint32_t nowMs, char* utcOut, size_t utcOutSize) {
  if (gnssUtcParsed) {
    const uint32_t baseCentis = gnssUtcCentis;
    const uint32_t baseMs = gnssUtcAdvanceMs;
    if (nowMs - baseMs > GGA_FRESH_MAX_MS) return false;
    const uint32_t elapsedMs = nowMs - baseMs;
    const uint32_t advancedCentis = baseCentis + (elapsedMs / 10u);
    formatUtcFromCentis(advancedCentis, utcOut, utcOutSize);
    return true;
  }

  if (gnssUtcRaw[0] != '\0') {
    if (nowMs - lastGgaMs > GGA_FRESH_MAX_MS) return false;
    strncpy(utcOut, gnssUtcRaw, utcOutSize - 1);
    utcOut[utcOutSize - 1] = '\0';
    return true;
  }

  return false;
}

bool gnssFixIsFresh(uint32_t nowMs) {
  return gnssFix && ((nowMs - lastGgaMs) <= GGA_FRESH_MAX_MS);
}

void updateStopState(uint32_t nowMs) {
  const bool speedFresh = gnssSpeedValid && ((nowMs - gnssSpeedMsTimestamp) <= SPEED_FRESH_MAX_MS);
  if (!speedFresh) {
    stopCandidateSince = 0;
    moveCandidateSince = 0;
    isStopped = false;
    return;
  }

  if (!isStopped) {
    if (gnssSpeedMS <= STOP_SPEED_MS_ENTER) {
      if (stopCandidateSince == 0) stopCandidateSince = nowMs;
      if (nowMs - stopCandidateSince >= STOP_CONFIRM_MS) {
        isStopped = true;
        moveCandidateSince = 0;
      }
    } else {
      stopCandidateSince = 0;
    }
  } else {
    if (gnssSpeedMS >= STOP_SPEED_MS_EXIT) {
      if (moveCandidateSince == 0) moveCandidateSince = nowMs;
      if (nowMs - moveCandidateSince >= STOP_CONFIRM_MS) {
        isStopped = false;
        stopCandidateSince = 0;
        avgClear();
      }
    } else {
      moveCandidateSince = 0;
    }
  }
}

void sendAt10Hz(uint32_t nowMs) {
  if (nowMs - lastOutMs < OUT_PERIOD_MS) return;
  lastOutMs = nowMs;

  if (!gnssFixIsFresh(nowMs)) return;

  double outLat = NAN;
  double outLon = NAN;
  const char* mode = nullptr;

  if (isStopped && avgHas) {
    outLat = avgLat;
    outLon = avgLon;
    mode = "AVG15s";
  } else if (hasOutputNow) {
    outLat = outputNowLat;
    outLon = outputNowLon;
    mode = "INST";
  } else {
    return;
  }

  char utcOut[16];
  if (!computeUtcForOutput(nowMs, utcOut, sizeof(utcOut))) return;

  char gpgga[220];
  if (!buildCorrectedGPGGA(gpgga, sizeof(gpgga), utcOut, outLat, outLon, gnssFixQ, gnssSats, gnssAlt)) return;

  OUT.print(gpgga);
  OUT.print("\r\n");
  outCount++;
  autoTest.ggaOut++;

  if (nowMs - lastLogMs > 500) {
    Serial.printf("OUT[%s] lat=%.8f lon=%.8f spd=%s%.3f stop=%s cnt=%lu badNMEA=%lu\n",
                  mode,
                  outLat,
                  outLon,
                  gnssSpeedValid ? "" : "invalid/",
                  gnssSpeedValid ? gnssSpeedMS : -1.0,
                  isStopped ? "true" : "false",
                  static_cast<unsigned long>(outCount),
                  static_cast<unsigned long>(malformedNmeaCount));
    Serial.println(gpgga);
    lastLogMs = nowMs;
  }
}

void updateAutoTest(uint32_t nowMs) {
  if (!autoTest.active) return;
  if (nowMs - autoTest.startMs < 10000) return;

  autoTest.active = false;

  Serial.println("[autotest] --- RESULTADOS (no bloqueante, 10s) ---");
  Serial.printf("[autotest] MAG init: %s\n", magOk ? "OK" : "FAIL");
  Serial.printf("[autotest] MAG lecturas: %lu\n", static_cast<unsigned long>(autoTest.magReads));
  Serial.printf("[autotest] GNSS GGA IN: %lu\n", static_cast<unsigned long>(autoTest.ggaIn));
  Serial.printf("[autotest] GNSS FIX detectado: %s\n", autoTest.gnssAnyFix ? "SI" : "NO");
  Serial.printf("[autotest] GPGGA OUT (MAX3232): %lu\n", static_cast<unsigned long>(autoTest.ggaOut));

  const bool pass = (autoTest.ggaIn > 0 && autoTest.ggaOut > 0);
  Serial.printf("[autotest] ESTADO: %s\n\n", pass ? "PASS" : "REVISAR CABLEADO/BAUD/PINES");
  Serial.println("[autotest] Nota: PASS valida flujo GNSS->OUT, heading MAG puede recuperarse en marcha.");
}

void processNmeaSentence(const char* line, uint32_t nowMs) {
  if (!line || line[0] != '$' || !verifyNmeaChecksum(line)) {
    malformedNmeaCount++;
    return;
  }

  bool parsedSpeedValid = false;
  double parsedSpeedMs = 0.0;
  bool parsedCogValid = false;
  double parsedCogDeg = 0.0;

  bool vtgSpeedValid = false;
  double vtgSpeedMs = 0.0;
  bool vtgCogValid = false;
  double vtgCogDeg = 0.0;
  const bool isVtg = parseVTG(line, vtgSpeedValid, vtgSpeedMs, vtgCogValid, vtgCogDeg);

  bool rmcSpeedValid = false;
  double rmcSpeedMs = 0.0;
  bool rmcCogValid = false;
  double rmcCogDeg = 0.0;
  const bool isRmc = parseRMC(line, rmcSpeedValid, rmcSpeedMs, rmcCogValid, rmcCogDeg);

  if (isVtg) {
    parsedSpeedValid = vtgSpeedValid;
    parsedSpeedMs = vtgSpeedMs;
    parsedCogValid = vtgCogValid;
    parsedCogDeg = vtgCogDeg;
  } else if (isRmc) {
    parsedSpeedValid = rmcSpeedValid;
    parsedSpeedMs = rmcSpeedMs;
    parsedCogValid = rmcCogValid;
    parsedCogDeg = rmcCogDeg;
  }

  if (isVtg || isRmc) {
    if (parsedSpeedValid) {
      gnssSpeedMS = parsedSpeedMs;
      gnssSpeedValid = true;
      gnssSpeedMsTimestamp = nowMs;
    }

    if (parsedCogValid) {
      lastCogDeg = parsedCogDeg;
      hasLastCog = true;
      lastCogMs = nowMs;
    }
  }

  GgaData gga;
  if (!parseGGA(line, gga) || !gga.valid) {
    return;
  }

  ggaCount++;
  autoTest.ggaIn++;
  lastGgaMs = nowMs;

  gnssFixQ = gga.fixQ;
  gnssFix = (gga.fixQ > 0);
  gnssLat = gga.lat;
  gnssLon = gga.lon;
  gnssAlt = gga.alt;
  gnssSats = gga.sats;

  strncpy(gnssUtcRaw, gga.utc, sizeof(gnssUtcRaw) - 1);
  gnssUtcRaw[sizeof(gnssUtcRaw) - 1] = '\0';
  gnssUtcParsed = parseUtcToCentis(gnssUtcRaw, gnssUtcCentis);
  if (gnssUtcParsed) {
    gnssUtcAdvanceMs = nowMs;
  } else {
    gnssUtcCentis = 0;
    gnssUtcAdvanceMs = nowMs;
  }

  if (gnssFix) autoTest.gnssAnyFix = true;

  if (!gnssFix) {
    hasOutputNow = false;
    return;
  }

  double headingTrue = NAN;
  const bool headingAvailable = resolveHeadingTrue(nowMs, headingTrue);

  double correctedLat = gnssLat;
  double correctedLon = gnssLon;
  applyOffsetFromAntennaToReference(gnssLat, gnssLon, headingAvailable, headingTrue, correctedLat, correctedLon);

  outputNowLat = correctedLat;
  outputNowLon = correctedLon;
  hasOutputNow = true;

  avgAdd(correctedLat, correctedLon, nowMs);
  avgCompute15s(nowMs);
}

// ================== SETUP =============================
void setup() {
  Serial.begin(115200);
  delay(20);

  setupLeds();

  GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  OUT.begin(OUT_BAUD, SERIAL_8N1, OUT_RX_PIN, OUT_TX_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  magOk = initMagnetometerQMC();
  magError = !magOk;

  autoTest.active = true;
  autoTest.startMs = millis();

  Serial.println();
  Serial.println("[boot] .ino unificado listo");
  Serial.printf("[boot] GNSS IN  RX=%d TX=%d @%lu\n", GNSS_RX_PIN, GNSS_TX_PIN, static_cast<unsigned long>(GNSS_BAUD));
  Serial.printf("[boot] RS232 OUT RX=%d TX=%d @%lu (MAX3232/Dynatest Embedded)\n",
                OUT_RX_PIN, OUT_TX_PIN, static_cast<unsigned long>(OUT_BAUD));
  Serial.printf("[boot] I2C SDA=%d SCL=%d MAG=0x%02X init=%s\n",
                I2C_SDA_PIN, I2C_SCL_PIN, MAG_ADDR, magOk ? "OK" : "FAIL");
  Serial.printf("[boot] offset along=%.2fm lateral=%.2fm decl=%.2f\n",
                offsetAlongHeadingMeters, offsetLateralMeters, declinationDeg);
  Serial.printf("[boot] stop enter<=%.2f m/s exit>=%.2f m/s win=%lums out=%lums\n",
                STOP_SPEED_MS_ENTER, STOP_SPEED_MS_EXIT,
                static_cast<unsigned long>(AVG_WINDOW_MS),
                static_cast<unsigned long>(OUT_PERIOD_MS));
  Serial.println("[boot] autotest no bloqueante activo durante los primeros 10s");
}

// ================== LOOP ==============================
void loop() {
  uint32_t now = millis();

  ensureMagRecovery(now);
  updateHeadingFromMag(now);

  char line[NMEA_LINE_MAX];
  while (nextNmeaLineFromGnss(line, sizeof(line))) {
    now = millis();
    processNmeaSentence(line, now);
  }

  now = millis();
  updateStopState(now);
  sendAt10Hz(now);
  updateAutoTest(now);
  updateLeds(now);
}
