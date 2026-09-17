#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>

static constexpr uint32_t GNSS_BAUD = 115200;
static constexpr uint32_t DYNATEST_BAUD = 38400;

static constexpr int GNSS_RX_PIN = 44;
static constexpr int GNSS_TX_PIN = 43;
static constexpr int DYNATEST_RX_PIN = 18;
static constexpr int DYNATEST_TX_PIN = 17;

static constexpr int IMU_SDA_PIN = 8;
static constexpr int IMU_SCL_PIN = 9;
static constexpr uint8_t IMU_I2C_ADDRESS = 0x4A;
static constexpr int IMU_RESET_PIN = -1;

static constexpr int LED_GREEN_PIN = 4;
static constexpr int LED_RED_PIN = 5;

static constexpr double ANTENNA_TO_PISTON_METERS = 1.50;
static constexpr double STOP_ENTER_SPEED_MS = 0.20;
static constexpr double STOP_EXIT_SPEED_MS = 0.30;
static constexpr uint32_t STOP_CONFIRM_DURATION_MS = 2000;
static constexpr uint32_t AVERAGING_DURATION_MS = 15000;
static constexpr uint32_t GGA_OUTPUT_PERIOD_MS = 100;
static constexpr uint32_t GNSS_FRESHNESS_MS = 1500;
static constexpr uint32_t SPEED_FRESHNESS_MS = 3000;
static constexpr double LOCK_DRIFT_THRESHOLD_METERS = 1.00;

static constexpr uint32_t IMU_REPORT_INTERVAL_US = 50000;
static constexpr uint32_t IMU_RETRY_INTERVAL_MS = 5000;
static constexpr uint32_t IMU_LOG_INTERVAL_MS = 3000;

static constexpr uint32_t DEBUG_LOG_INTERVAL_MS = 5000;
static constexpr uint32_t PPP_LOG_INTERVAL_MS = 2000;

static constexpr size_t GNSS_LINE_BUFFER_SIZE = 384;
static constexpr size_t GGA_BODY_BUFFER_SIZE = 160;
static constexpr size_t RMC_BODY_BUFFER_SIZE = 128;
static constexpr size_t FIELD_COUNT = 20;
static constexpr size_t UTC_BUFFER_SIZE = 16;
static constexpr size_t MAX_SAMPLES = 160;

enum PppState {
  SIN_PPP,
  PPP_CONVERGING,
  PPP_ESTABLE
};

enum MotionState {
  MOVING,
  STOP_CONFIRM,
  AVERAGING,
  LOCKED
};

enum LedPattern {
  LED_PATTERN_OFF,
  LED_PATTERN_SOLID,
  LED_PATTERN_BLINK_SLOW,
  LED_PATTERN_BLINK_FAST,
  LED_PATTERN_DOUBLE_BLINK
};

struct GnssState {
  bool valid;
  double lat;
  double lon;
  double alt;
  double hdop;
  double geoidSeparation;
  double speedMs;
  double cogDeg;
  uint8_t fixQuality;
  uint8_t satellites;
  char utc[UTC_BUFFER_SIZE];
  uint32_t lastGgaMs;
  uint32_t lastSpeedMs;
};

struct LockedState {
  bool valid;
  double correctedLat;
  double correctedLon;
  double correctedAlt;
  double yawDeg;
  double referenceLat;
  double referenceLon;
  double referenceAlt;
};

void setup();
void loop();
void initializeGnssState();
void initializeLockedState();
void updateImu(uint32_t nowMs);
bool initializeImu(uint32_t nowMs);
bool ensureImuReportEnabled(uint32_t nowMs);
bool quaternionToYawDegrees(const sh2_RotationVectorWAcc_t &rotationVector, double &yawDeg);
void readGnss(uint32_t nowMs);
void resetInputLine();
void processGnssLine(const char *line, uint32_t nowMs);
bool parseGGA(const char *line, GnssState &outState, uint32_t nowMs);
bool parseRMC(const char *line, GnssState &state, uint32_t nowMs);
void parsePPPNAV(const char *line, uint32_t nowMs);
bool extractNmeaBody(const char *line, char *body, size_t bodySize);
bool validateNmeaChecksum(const char *line, const char *star);
bool parseUtcField(const char *text, char *utcOut, size_t utcOutSize);
bool parseNmeaCoordinate(const char *value, const char *hemisphere, bool isLatitude, double &degreesOut);
bool parseStrictDouble(const char *text, double &valueOut);
bool parseStrictInt(const char *text, long &valueOut);
size_t splitCsvFields(char *text, char *fields[], size_t maxFields);
void copyCString(char *destination, size_t destinationSize, const char *source);
bool hasFreshGnssFix(uint32_t nowMs);
bool hasFreshSpeed(uint32_t nowMs);
void updateMotionState(uint32_t nowMs);
void setMotionState(MotionState newState);
void startAveraging(uint32_t nowMs);
void appendAverageSampleIfNeeded();
void finishAveraging();
void clearAverageSamples();
void clearLockedState();
void linearizeSamples(const double *source, double *destination, size_t count);
double trimmedMean(const double *values, size_t count);
double circularMeanDegrees(const double *values, size_t count);
double haversineMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
bool detectLockedDrift();
void applyAntennaOffset(double latIn, double lonIn, double yawDeg, double &latOut, double &lonOut);
void destinationPoint(double latDeg, double lonDeg, double bearingDeg, double distanceMeters, double &latOutDeg, double &lonOutDeg);
double normalizeDegrees(double degrees);
uint8_t deriveOutputFixQuality();
bool buildOutputGga(char *buffer, size_t bufferSize, double lat, double lon, double alt, uint8_t fixQuality, uint8_t satellites, const char *utc, double hdop, double geoidSeparation);
void formatNmeaCoordinate(double decimalDegrees, bool isLatitude, char *valueOut, size_t valueSize, char &hemisphereOut);
uint8_t calculateNmeaChecksum(const char *payload);
void transmitDynatest(uint32_t nowMs);
void updateLeds(uint32_t nowMs);
bool ledPatternIsOn(LedPattern pattern, uint32_t nowMs);
const char *pppStateName(PppState state);
const char *motionStateName(MotionState state);

HardwareSerial GNSS(1);
HardwareSerial Dynatest(2);
TwoWire ImuWire(0);
Adafruit_BNO08x bno08x(IMU_RESET_PIN);
sh2_SensorValue_t bnoEvent;

GnssState gnssState;
LockedState lockedState;

PppState pppState = SIN_PPP;
MotionState motionState = MOVING;

double currentYaw = NAN;
bool imuPresent = false;
bool imuReportEnabled = false;
uint32_t lastImuAttemptMs = 0;
uint32_t lastImuLogMs = 0;
uint32_t lastYawUpdateMs = 0;

char inputLine[GNSS_LINE_BUFFER_SIZE];
size_t inputLength = 0;
bool inputCapturing = false;
bool inputOverflow = false;
uint32_t lastOverflowLogMs = 0;

double latSamples[MAX_SAMPLES];
double lonSamples[MAX_SAMPLES];
double altSamples[MAX_SAMPLES];
double yawSamples[MAX_SAMPLES];
size_t sampleCount = 0;
size_t sampleWriteIndex = 0;
uint32_t averagingStartedMs = 0;
uint32_t stopCandidateSinceMs = 0;
uint32_t lastSampledGgaMs = 0;

uint32_t lastOutputMs = 0;
uint32_t lastDebugLogMs = 0;
uint32_t lastUnknownPppLogMs = 0;

void initializeGnssState() {
  gnssState.valid = false;
  gnssState.lat = NAN;
  gnssState.lon = NAN;
  gnssState.alt = NAN;
  gnssState.hdop = NAN;
  gnssState.geoidSeparation = NAN;
  gnssState.speedMs = NAN;
  gnssState.cogDeg = NAN;
  gnssState.fixQuality = 0;
  gnssState.satellites = 0;
  gnssState.utc[0] = '\0';
  gnssState.lastGgaMs = 0;
  gnssState.lastSpeedMs = 0;
}

void initializeLockedState() {
  lockedState.valid = false;
  lockedState.correctedLat = NAN;
  lockedState.correctedLon = NAN;
  lockedState.correctedAlt = NAN;
  lockedState.yawDeg = NAN;
  lockedState.referenceLat = NAN;
  lockedState.referenceLon = NAN;
  lockedState.referenceAlt = NAN;
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  initializeGnssState();
  initializeLockedState();
  clearAverageSamples();
  resetInputLine();

  GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Dynatest.begin(DYNATEST_BAUD, SERIAL_8N1, DYNATEST_RX_PIN, DYNATEST_TX_PIN);

  ImuWire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 100000U);
  initializeImu(millis());

  Serial.println(F("[boot] FWMGPSV89 listo"));
  Serial.printf("[boot] GNSS Serial1 RX=%d TX=%d @%lu\n",
                GNSS_RX_PIN, GNSS_TX_PIN, static_cast<unsigned long>(GNSS_BAUD));
  Serial.printf("[boot] Dynatest Serial2 RX=%d TX=%d @%lu\n",
                DYNATEST_RX_PIN, DYNATEST_TX_PIN, static_cast<unsigned long>(DYNATEST_BAUD));
  Serial.printf("[boot] I2C IMU SDA=%d SCL=%d addr=0x%02X\n",
                IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_ADDRESS);
}

void loop() {
  const uint32_t nowMs = millis();

  updateImu(nowMs);
  readGnss(nowMs);
  updateMotionState(nowMs);
  transmitDynatest(nowMs);
  updateLeds(nowMs);

  if ((nowMs - lastDebugLogMs) >= DEBUG_LOG_INTERVAL_MS) {
    lastDebugLogMs = nowMs;
    Serial.printf("[state] %s | PPP=%s | fix=%u sats=%u yaw=%s\n",
                  motionStateName(motionState),
                  pppStateName(pppState),
                  gnssState.fixQuality,
                  gnssState.satellites,
                  std::isfinite(currentYaw) ? "OK" : "NAN");
  }
}

bool initializeImu(uint32_t nowMs) {
  lastImuAttemptMs = nowMs;
  imuReportEnabled = false;
  currentYaw = NAN;

  if (!bno08x.begin_I2C(IMU_I2C_ADDRESS, &ImuWire)) {
    imuPresent = false;
    if ((nowMs - lastImuLogMs) >= IMU_LOG_INTERVAL_MS) {
      lastImuLogMs = nowMs;
      Serial.println(F("[imu] BNO085 no detectado; continúa sin yaw"));
    }
    return false;
  }

  imuPresent = true;
  Serial.println(F("[imu] BNO085 detectado"));
  return ensureImuReportEnabled(nowMs);
}

bool ensureImuReportEnabled(uint32_t nowMs) {
  if (!imuPresent) {
    return false;
  }

  if (bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
    imuReportEnabled = true;
    return true;
  }

  imuReportEnabled = false;
  if ((nowMs - lastImuLogMs) >= IMU_LOG_INTERVAL_MS) {
    lastImuLogMs = nowMs;
    Serial.println(F("[imu] enableReport(SH2_ROTATION_VECTOR) falló"));
  }
  return false;
}

void updateImu(uint32_t nowMs) {
  if (!imuPresent) {
    if ((nowMs - lastImuAttemptMs) >= IMU_RETRY_INTERVAL_MS) {
      initializeImu(nowMs);
    }
    return;
  }

  if (bno08x.wasReset()) {
    imuReportEnabled = false;
    currentYaw = NAN;
    if ((nowMs - lastImuLogMs) >= IMU_LOG_INTERVAL_MS) {
      lastImuLogMs = nowMs;
      Serial.println(F("[imu] reset detectado; reconfigurando"));
    }
  }

  if (!imuReportEnabled && !ensureImuReportEnabled(nowMs)) {
    return;
  }

  bool gotYaw = false;
  while (bno08x.getSensorEvent(&bnoEvent)) {
    if (bnoEvent.sensorId != SH2_ROTATION_VECTOR) {
      continue;
    }

    double yawDeg = NAN;
    if (quaternionToYawDegrees(bnoEvent.un.rotationVector, yawDeg)) {
      currentYaw = normalizeDegrees(yawDeg);
      lastYawUpdateMs = nowMs;
      gotYaw = true;
    }
  }

  if (!gotYaw && (nowMs - lastYawUpdateMs) > (IMU_RETRY_INTERVAL_MS * 2U)) {
    currentYaw = NAN;
  }
}

bool quaternionToYawDegrees(const sh2_RotationVectorWAcc_t &rotationVector, double &yawDeg) {
  const double qr = rotationVector.real;
  const double qi = rotationVector.i;
  const double qj = rotationVector.j;
  const double qk = rotationVector.k;

  if (!std::isfinite(qr) || !std::isfinite(qi) || !std::isfinite(qj) || !std::isfinite(qk)) {
    yawDeg = NAN;
    return false;
  }

  const double siny = 2.0 * ((qr * qk) + (qi * qj));
  const double cosy = 1.0 - (2.0 * ((qj * qj) + (qk * qk)));
  yawDeg = atan2(siny, cosy) * 180.0 / PI;
  return std::isfinite(yawDeg);
}

void readGnss(uint32_t nowMs) {
  while (GNSS.available() > 0) {
    const char c = static_cast<char>(GNSS.read());

    if (c == '$' || c == '#') {
      resetInputLine();
      inputCapturing = true;
      inputLine[inputLength++] = c;
      continue;
    }

    if (!inputCapturing) {
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (inputLength > 0 && !inputOverflow) {
        inputLine[inputLength] = '\0';
        processGnssLine(inputLine, nowMs);
      }
      resetInputLine();
      continue;
    }

    if (inputLength < (GNSS_LINE_BUFFER_SIZE - 1U)) {
      inputLine[inputLength++] = c;
    } else {
      inputOverflow = true;
      if ((nowMs - lastOverflowLogMs) >= DEBUG_LOG_INTERVAL_MS) {
        lastOverflowLogMs = nowMs;
        Serial.println(F("[gnss] línea descartada por overflow"));
      }
    }
  }
}

void resetInputLine() {
  inputLength = 0;
  inputCapturing = false;
  inputOverflow = false;
  inputLine[0] = '\0';
}

void processGnssLine(const char *line, uint32_t nowMs) {
  if (line[0] == '$') {
    if ((strncmp(line, "$GPGGA", 6) == 0) || (strncmp(line, "$GNGGA", 6) == 0)) {
      GnssState parsed = gnssState;
      if (parseGGA(line, parsed, nowMs)) {
        gnssState = parsed;
      } else {
        gnssState.valid = false;
        gnssState.fixQuality = 0;
      }
      return;
    }

    if ((strncmp(line, "$GPRMC", 6) == 0) || (strncmp(line, "$GNRMC", 6) == 0)) {
      parseRMC(line, gnssState, nowMs);
    }
    return;
  }

  if (line[0] == '#') {
    parsePPPNAV(line, nowMs);
  }
}

bool parseGGA(const char *line, GnssState &outState, uint32_t nowMs) {
  char body[GGA_BODY_BUFFER_SIZE];
  if (!extractNmeaBody(line, body, sizeof(body))) {
    return false;
  }

  char *fields[FIELD_COUNT];
  const size_t fieldCount = splitCsvFields(body, fields, FIELD_COUNT);
  if (fieldCount < 13U) {
    return false;
  }

  if ((strcmp(fields[0], "GPGGA") != 0) && (strcmp(fields[0], "GNGGA") != 0)) {
    return false;
  }

  char utc[UTC_BUFFER_SIZE];
  if (!parseUtcField(fields[1], utc, sizeof(utc))) {
    return false;
  }

  long fixQuality = 0;
  long satellites = 0;
  if (!parseStrictInt(fields[6], fixQuality) || !parseStrictInt(fields[7], satellites)) {
    return false;
  }

  if (fixQuality < 1L || satellites < 0L) {
    return false;
  }

  double lat = NAN;
  double lon = NAN;
  double alt = NAN;
  double hdop = NAN;
  double geoidSeparation = NAN;

  if (!parseNmeaCoordinate(fields[2], fields[3], true, lat) ||
      !parseNmeaCoordinate(fields[4], fields[5], false, lon) ||
      !parseStrictDouble(fields[9], alt)) {
    return false;
  }

  if (fields[8][0] != '\0' && !parseStrictDouble(fields[8], hdop)) {
    return false;
  }

  if (fields[11][0] != '\0' && !parseStrictDouble(fields[11], geoidSeparation)) {
    return false;
  }

  outState.valid = true;
  outState.lat = lat;
  outState.lon = lon;
  outState.alt = alt;
  outState.hdop = hdop;
  outState.geoidSeparation = geoidSeparation;
  outState.fixQuality = static_cast<uint8_t>(fixQuality);
  outState.satellites = static_cast<uint8_t>(satellites);
  outState.lastGgaMs = nowMs;
  copyCString(outState.utc, sizeof(outState.utc), utc);
  return true;
}

bool parseRMC(const char *line, GnssState &state, uint32_t nowMs) {
  char body[RMC_BODY_BUFFER_SIZE];
  if (!extractNmeaBody(line, body, sizeof(body))) {
    return false;
  }

  char *fields[FIELD_COUNT];
  const size_t fieldCount = splitCsvFields(body, fields, FIELD_COUNT);
  if (fieldCount < 9U) {
    return false;
  }

  if ((strcmp(fields[0], "GPRMC") != 0) && (strcmp(fields[0], "GNRMC") != 0)) {
    return false;
  }

  if (fields[2][0] != 'A' || fields[2][1] != '\0') {
    return false;
  }

  double speedKnots = NAN;
  if (!parseStrictDouble(fields[7], speedKnots)) {
    return false;
  }

  double cogDeg = NAN;
  if (fields[8][0] != '\0' && !parseStrictDouble(fields[8], cogDeg)) {
    return false;
  }

  state.speedMs = speedKnots * 0.514444;
  state.cogDeg = cogDeg;
  state.lastSpeedMs = nowMs;
  return true;
}

void parsePPPNAV(const char *line, uint32_t nowMs) {
  if (strncmp(line, "#PPPNAVA", 8) != 0) {
    return;
  }

  if (strstr(line, "PPP_CONVERGING") != nullptr) {
    pppState = PPP_CONVERGING;
    return;
  }

  if ((strstr(line, "PPP_FIXED") != nullptr) || (strstr(line, "PPP_ESTABLE") != nullptr)) {
    pppState = PPP_ESTABLE;
    return;
  }

  if (strstr(line, "SINGLE") != nullptr) {
    pppState = SIN_PPP;
    return;
  }

  if ((nowMs - lastUnknownPppLogMs) >= PPP_LOG_INTERVAL_MS) {
    lastUnknownPppLogMs = nowMs;
    Serial.print(F("[ppp] línea sin clasificar: "));
    Serial.println(line);
  }
}

bool extractNmeaBody(const char *line, char *body, size_t bodySize) {
  if (line == nullptr || line[0] != '$') {
    return false;
  }

  const char *star = strchr(line, '*');
  if (star == nullptr || !validateNmeaChecksum(line, star)) {
    return false;
  }

  const size_t payloadLength = static_cast<size_t>(star - line - 1);
  if (payloadLength == 0U || payloadLength >= bodySize) {
    return false;
  }

  memcpy(body, line + 1, payloadLength);
  body[payloadLength] = '\0';
  return true;
}

bool validateNmeaChecksum(const char *line, const char *star) {
  if ((star - line) < 2 || star[1] == '\0' || star[2] == '\0') {
    return false;
  }

  uint8_t calculated = 0;
  for (const char *cursor = line + 1; cursor < star; ++cursor) {
    calculated ^= static_cast<uint8_t>(*cursor);
  }

  char expected[3] = {star[1], star[2], '\0'};
  char *endptr = nullptr;
  const long provided = strtol(expected, &endptr, 16);
  return (endptr != expected) && (*endptr == '\0') && (provided == static_cast<long>(calculated));
}

bool parseUtcField(const char *text, char *utcOut, size_t utcOutSize) {
  if (text == nullptr || text[0] == '\0') {
    return false;
  }

  size_t digitCount = 0;
  bool seenDecimal = false;
  for (const char *cursor = text; *cursor != '\0'; ++cursor) {
    if (isdigit(static_cast<unsigned char>(*cursor))) {
      ++digitCount;
      continue;
    }
    if (*cursor == '.' && !seenDecimal) {
      seenDecimal = true;
      continue;
    }
    return false;
  }

  if (digitCount < 6U || strlen(text) >= utcOutSize) {
    return false;
  }

  copyCString(utcOut, utcOutSize, text);
  return true;
}

bool parseNmeaCoordinate(const char *value, const char *hemisphere, bool isLatitude, double &degreesOut) {
  if (value == nullptr || hemisphere == nullptr || value[0] == '\0' || hemisphere[0] == '\0') {
    degreesOut = NAN;
    return false;
  }

  const size_t degDigits = isLatitude ? 2U : 3U;
  const size_t valueLength = strlen(value);
  const char *decimalPoint = strchr(value, '.');

  if (valueLength < (degDigits + 3U) || decimalPoint == nullptr || static_cast<size_t>(decimalPoint - value) < degDigits) {
    degreesOut = NAN;
    return false;
  }

  char degreesText[4] = {0};
  memcpy(degreesText, value, degDigits);

  char minutesText[16] = {0};
  copyCString(minutesText, sizeof(minutesText), value + degDigits);

  char *degreesEnd = nullptr;
  char *minutesEnd = nullptr;
  const long wholeDegrees = strtol(degreesText, &degreesEnd, 10);
  const double wholeMinutes = strtod(minutesText, &minutesEnd);

  if (*degreesEnd != '\0' || *minutesEnd != '\0' || wholeMinutes < 0.0 || wholeMinutes >= 60.0) {
    degreesOut = NAN;
    return false;
  }

  degreesOut = static_cast<double>(wholeDegrees) + (wholeMinutes / 60.0);

  if ((isLatitude && (hemisphere[0] == 'S')) || (!isLatitude && (hemisphere[0] == 'W'))) {
    degreesOut = -degreesOut;
  } else if (!((isLatitude && hemisphere[0] == 'N') || (!isLatitude && hemisphere[0] == 'E'))) {
    degreesOut = NAN;
    return false;
  }

  return std::isfinite(degreesOut);
}

bool parseStrictDouble(const char *text, double &valueOut) {
  if (text == nullptr || text[0] == '\0') {
    valueOut = NAN;
    return false;
  }

  char *endptr = nullptr;
  valueOut = strtod(text, &endptr);
  return (endptr != text) && (*endptr == '\0') && std::isfinite(valueOut);
}

bool parseStrictInt(const char *text, long &valueOut) {
  if (text == nullptr || text[0] == '\0') {
    valueOut = 0;
    return false;
  }

  char *endptr = nullptr;
  valueOut = strtol(text, &endptr, 10);
  return (endptr != text) && (*endptr == '\0');
}

size_t splitCsvFields(char *text, char *fields[], size_t maxFields) {
  size_t count = 0;
  fields[count++] = text;

  for (char *cursor = text; *cursor != '\0' && count < maxFields; ++cursor) {
    if (*cursor == ',') {
      *cursor = '\0';
      fields[count++] = cursor + 1;
    }
  }

  return count;
}

void copyCString(char *destination, size_t destinationSize, const char *source) {
  if (destination == nullptr || destinationSize == 0U) {
    return;
  }

  if (source == nullptr) {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, destinationSize - 1U);
  destination[destinationSize - 1U] = '\0';
}

bool hasFreshGnssFix(uint32_t nowMs) {
  return gnssState.valid && ((nowMs - gnssState.lastGgaMs) <= GNSS_FRESHNESS_MS);
}

bool hasFreshSpeed(uint32_t nowMs) {
  return std::isfinite(gnssState.speedMs) && ((nowMs - gnssState.lastSpeedMs) <= SPEED_FRESHNESS_MS);
}

void updateMotionState(uint32_t nowMs) {
  const bool speedFresh = hasFreshSpeed(nowMs);
  const bool belowEnter = speedFresh && (gnssState.speedMs < STOP_ENTER_SPEED_MS);
  const bool aboveExit = speedFresh && (gnssState.speedMs > STOP_EXIT_SPEED_MS);

  switch (motionState) {
    case MOVING:
      if (belowEnter) {
        stopCandidateSinceMs = nowMs;
        setMotionState(STOP_CONFIRM);
      }
      break;

    case STOP_CONFIRM:
      if (!speedFresh) {
        stopCandidateSinceMs = 0;
        setMotionState(MOVING);
      } else if (aboveExit) {
        stopCandidateSinceMs = 0;
        setMotionState(MOVING);
      } else if ((nowMs - stopCandidateSinceMs) >= STOP_CONFIRM_DURATION_MS) {
        startAveraging(nowMs);
      }
      break;

    case AVERAGING:
      if (aboveExit) {
        clearAverageSamples();
        clearLockedState();
        setMotionState(MOVING);
      } else {
        appendAverageSampleIfNeeded();
        if ((nowMs - averagingStartedMs) >= AVERAGING_DURATION_MS) {
          finishAveraging();
        }
      }
      break;

    case LOCKED:
      if (aboveExit || detectLockedDrift()) {
        clearAverageSamples();
        clearLockedState();
        setMotionState(MOVING);
      }
      break;
  }
}

void setMotionState(MotionState newState) {
  if (motionState == newState) {
    return;
  }

  const MotionState oldState = motionState;
  motionState = newState;

  if ((oldState == AVERAGING || oldState == LOCKED) && (newState != LOCKED)) {
    clearAverageSamples();
    clearLockedState();
  }

  Serial.printf("[state] %s -> %s\n", motionStateName(oldState), motionStateName(newState));
}

void startAveraging(uint32_t nowMs) {
  clearAverageSamples();
  clearLockedState();
  averagingStartedMs = nowMs;
  lastSampledGgaMs = 0;
  setMotionState(AVERAGING);
}

void appendAverageSampleIfNeeded() {
  if (!gnssState.valid || gnssState.lastGgaMs == 0 || gnssState.lastGgaMs == lastSampledGgaMs) {
    return;
  }

  latSamples[sampleWriteIndex] = gnssState.lat;
  lonSamples[sampleWriteIndex] = gnssState.lon;
  altSamples[sampleWriteIndex] = gnssState.alt;
  yawSamples[sampleWriteIndex] = currentYaw;

  sampleWriteIndex = (sampleWriteIndex + 1U) % MAX_SAMPLES;
  if (sampleCount < MAX_SAMPLES) {
    ++sampleCount;
  }

  lastSampledGgaMs = gnssState.lastGgaMs;
}

void finishAveraging() {
  double orderedLat[MAX_SAMPLES];
  double orderedLon[MAX_SAMPLES];
  double orderedAlt[MAX_SAMPLES];
  double orderedYaw[MAX_SAMPLES];

  linearizeSamples(latSamples, orderedLat, sampleCount);
  linearizeSamples(lonSamples, orderedLon, sampleCount);
  linearizeSamples(altSamples, orderedAlt, sampleCount);
  linearizeSamples(yawSamples, orderedYaw, sampleCount);

  const double meanLat = trimmedMean(orderedLat, sampleCount);
  const double meanLon = trimmedMean(orderedLon, sampleCount);
  const double meanAlt = trimmedMean(orderedAlt, sampleCount);
  const double meanYaw = circularMeanDegrees(orderedYaw, sampleCount);

  if (!std::isfinite(meanLat) || !std::isfinite(meanLon) || !std::isfinite(meanAlt)) {
    Serial.println(F("[avg] medias inválidas; reiniciando ventana"));
    averagingStartedMs = millis();
    clearAverageSamples();
    return;
  }

  lockedState.referenceLat = meanLat;
  lockedState.referenceLon = meanLon;
  lockedState.referenceAlt = meanAlt;
  lockedState.correctedAlt = meanAlt;
  lockedState.yawDeg = meanYaw;

  if (std::isfinite(meanYaw)) {
    applyAntennaOffset(meanLat, meanLon, meanYaw, lockedState.correctedLat, lockedState.correctedLon);
  } else {
    lockedState.correctedLat = meanLat;
    lockedState.correctedLon = meanLon;
  }

  lockedState.valid = std::isfinite(lockedState.correctedLat) && std::isfinite(lockedState.correctedLon);
  if (!lockedState.valid) {
    Serial.println(F("[avg] lock inválido; reiniciando ventana"));
    averagingStartedMs = millis();
    clearAverageSamples();
    return;
  }

  setMotionState(LOCKED);
}

void clearAverageSamples() {
  sampleCount = 0;
  sampleWriteIndex = 0;
  averagingStartedMs = 0;
  lastSampledGgaMs = 0;
}

void clearLockedState() {
  initializeLockedState();
}

void linearizeSamples(const double *source, double *destination, size_t count) {
  if (count == 0U) {
    return;
  }

  size_t startIndex = 0U;
  if (count >= MAX_SAMPLES) {
    startIndex = sampleWriteIndex;
  }

  for (size_t i = 0; i < count; ++i) {
    destination[i] = source[(startIndex + i) % MAX_SAMPLES];
  }
}

double trimmedMean(const double *values, size_t count) {
  if (count == 0U) {
    return NAN;
  }

  double filtered[MAX_SAMPLES];
  size_t validCount = 0;
  for (size_t i = 0; i < count && i < MAX_SAMPLES; ++i) {
    if (std::isfinite(values[i])) {
      filtered[validCount++] = values[i];
    }
  }

  if (validCount == 0U) {
    return NAN;
  }

  std::sort(filtered, filtered + validCount);

  size_t trimCount = 0;
  if (validCount >= 10U) {
    trimCount = validCount / 10U;
  }

  if ((trimCount * 2U) >= validCount) {
    trimCount = 0;
  }

  double sum = 0.0;
  size_t used = 0;
  for (size_t i = trimCount; i < (validCount - trimCount); ++i) {
    sum += filtered[i];
    ++used;
  }

  return (used > 0U) ? (sum / static_cast<double>(used)) : NAN;
}

double circularMeanDegrees(const double *values, size_t count) {
  if (count == 0U) {
    return NAN;
  }

  double sumSin = 0.0;
  double sumCos = 0.0;
  size_t validCount = 0;
  for (size_t i = 0; i < count && i < MAX_SAMPLES; ++i) {
    if (!std::isfinite(values[i])) {
      continue;
    }
    const double radiansValue = values[i] * DEG_TO_RAD;
    sumSin += sin(radiansValue);
    sumCos += cos(radiansValue);
    ++validCount;
  }

  if (validCount == 0U || !std::isfinite(sumSin) || !std::isfinite(sumCos)) {
    return NAN;
  }

  return normalizeDegrees(atan2(sumSin, sumCos) * 180.0 / PI);
}

double haversineMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
  static constexpr double EarthRadiusMeters = 6378137.0;

  const double dLat = radians(lat2Deg - lat1Deg);
  const double dLon = radians(lon2Deg - lon1Deg);
  const double lat1Rad = radians(lat1Deg);
  const double lat2Rad = radians(lat2Deg);

  const double a = sq(sin(dLat / 2.0)) +
                   (cos(lat1Rad) * cos(lat2Rad) * sq(sin(dLon / 2.0)));
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EarthRadiusMeters * c;
}

bool detectLockedDrift() {
  if (!lockedState.valid || !gnssState.valid) {
    return false;
  }

  const double driftMeters = haversineMeters(
      lockedState.referenceLat, lockedState.referenceLon,
      gnssState.lat, gnssState.lon);

  return std::isfinite(driftMeters) && (driftMeters > LOCK_DRIFT_THRESHOLD_METERS);
}

void applyAntennaOffset(double latIn, double lonIn, double yawDeg, double &latOut, double &lonOut) {
  if (!std::isfinite(yawDeg)) {
    latOut = latIn;
    lonOut = lonIn;
    return;
  }

  const double bearingDeg = normalizeDegrees(yawDeg + 180.0);
  destinationPoint(latIn, lonIn, bearingDeg, ANTENNA_TO_PISTON_METERS, latOut, lonOut);
}

void destinationPoint(double latDeg, double lonDeg, double bearingDeg, double distanceMeters, double &latOutDeg, double &lonOutDeg) {
  static constexpr double EarthRadiusMeters = 6378137.0;

  const double angularDistance = distanceMeters / EarthRadiusMeters;
  const double bearingRad = radians(bearingDeg);
  const double lat1 = radians(latDeg);
  const double lon1 = radians(lonDeg);

  const double sinLat1 = sin(lat1);
  const double cosLat1 = cos(lat1);
  const double sinAd = sin(angularDistance);
  const double cosAd = cos(angularDistance);

  const double lat2 = asin((sinLat1 * cosAd) + (cosLat1 * sinAd * cos(bearingRad)));
  const double lon2 = lon1 + atan2(
      sin(bearingRad) * sinAd * cosLat1,
      cosAd - (sinLat1 * sin(lat2)));

  latOutDeg = degrees(lat2);
  lonOutDeg = degrees(lon2);
}

double normalizeDegrees(double degreesValue) {
  while (degreesValue < 0.0) {
    degreesValue += 360.0;
  }
  while (degreesValue >= 360.0) {
    degreesValue -= 360.0;
  }
  return degreesValue;
}

uint8_t deriveOutputFixQuality() {
  uint8_t derived = gnssState.fixQuality;
  if (derived == 0U && gnssState.valid) {
    derived = 1U;
  }

  if (pppState == PPP_ESTABLE && derived < 4U) {
    derived = 4U;
  } else if (pppState == PPP_CONVERGING && derived < 2U) {
    derived = 2U;
  }

  return derived;
}

bool buildOutputGga(char *buffer, size_t bufferSize, double lat, double lon, double alt, uint8_t fixQuality, uint8_t satellites, const char *utc, double hdop, double geoidSeparation) {
  if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) || utc == nullptr || utc[0] == '\0') {
    return false;
  }

  char latValue[16];
  char lonValue[16];
  char hdopValue[16];
  char altValue[16];
  char geoidValue[16];
  char ns = 'N';
  char ew = 'E';

  formatNmeaCoordinate(lat, true, latValue, sizeof(latValue), ns);
  formatNmeaCoordinate(lon, false, lonValue, sizeof(lonValue), ew);

  if (std::isfinite(hdop)) {
    snprintf(hdopValue, sizeof(hdopValue), "%.1f", hdop);
  } else {
    hdopValue[0] = '\0';
  }

  snprintf(altValue, sizeof(altValue), "%.2f", alt);

  if (std::isfinite(geoidSeparation)) {
    snprintf(geoidValue, sizeof(geoidValue), "%.2f", geoidSeparation);
  } else {
    geoidValue[0] = '\0';
  }

  char payload[128];
  const int payloadLength = snprintf(
      payload, sizeof(payload),
      "GPGGA,%s,%s,%c,%s,%c,%u,%02u,%s,%s,M,%s,M,,",
      utc,
      latValue, ns,
      lonValue, ew,
      static_cast<unsigned>(fixQuality),
      static_cast<unsigned>(satellites),
      hdopValue,
      altValue,
      geoidValue);

  if (payloadLength <= 0 || static_cast<size_t>(payloadLength) >= sizeof(payload)) {
    return false;
  }

  const uint8_t checksum = calculateNmeaChecksum(payload);
  const int sentenceLength = snprintf(buffer, bufferSize, "$%s*%02X", payload, checksum);
  return sentenceLength > 0 && static_cast<size_t>(sentenceLength) < bufferSize;
}

void formatNmeaCoordinate(double decimalDegrees, bool isLatitude, char *valueOut, size_t valueSize, char &hemisphereOut) {
  const double absoluteDegrees = fabs(decimalDegrees);
  const unsigned wholeDegrees = static_cast<unsigned>(absoluteDegrees);
  const double minutes = (absoluteDegrees - static_cast<double>(wholeDegrees)) * 60.0;

  if (isLatitude) {
    hemisphereOut = (decimalDegrees >= 0.0) ? 'N' : 'S';
    snprintf(valueOut, valueSize, "%02u%07.4f", wholeDegrees, minutes);
  } else {
    hemisphereOut = (decimalDegrees >= 0.0) ? 'E' : 'W';
    snprintf(valueOut, valueSize, "%03u%07.4f", wholeDegrees, minutes);
  }
}

uint8_t calculateNmeaChecksum(const char *payload) {
  uint8_t checksum = 0;
  for (const char *cursor = payload; *cursor != '\0'; ++cursor) {
    checksum ^= static_cast<uint8_t>(*cursor);
  }
  return checksum;
}

void transmitDynatest(uint32_t nowMs) {
  if (lastOutputMs == 0U) {
    lastOutputMs = nowMs;
    return;
  }

  if ((nowMs - lastOutputMs) < GGA_OUTPUT_PERIOD_MS) {
    return;
  }
  lastOutputMs += GGA_OUTPUT_PERIOD_MS;
  if ((nowMs - lastOutputMs) >= GGA_OUTPUT_PERIOD_MS) {
    lastOutputMs = nowMs;
  }

  if (!hasFreshGnssFix(nowMs)) {
    return;
  }

  double outputLat = gnssState.lat;
  double outputLon = gnssState.lon;
  double outputAlt = gnssState.alt;

  if (motionState == LOCKED) {
    if (!lockedState.valid) {
      return;
    }
    outputLat = lockedState.correctedLat;
    outputLon = lockedState.correctedLon;
    outputAlt = lockedState.correctedAlt;
  } else {
    applyAntennaOffset(gnssState.lat, gnssState.lon, currentYaw, outputLat, outputLon);
  }

  char ggaSentence[160];
  if (!buildOutputGga(
          ggaSentence, sizeof(ggaSentence),
          outputLat, outputLon, outputAlt,
          deriveOutputFixQuality(),
          gnssState.satellites,
          gnssState.utc,
          gnssState.hdop,
          gnssState.geoidSeparation)) {
    return;
  }

  Dynatest.println(ggaSentence);
}

void updateLeds(uint32_t nowMs) {
  LedPattern redPattern = LED_PATTERN_OFF;
  if (hasFreshGnssFix(nowMs)) {
    if (pppState == PPP_ESTABLE) {
      redPattern = LED_PATTERN_SOLID;
    } else if (pppState == PPP_CONVERGING) {
      redPattern = LED_PATTERN_DOUBLE_BLINK;
    } else {
      redPattern = LED_PATTERN_BLINK_SLOW;
    }
  }

  LedPattern greenPattern = LED_PATTERN_BLINK_SLOW;
  if (motionState == STOP_CONFIRM) {
    greenPattern = LED_PATTERN_DOUBLE_BLINK;
  } else if (motionState == AVERAGING) {
    greenPattern = LED_PATTERN_BLINK_FAST;
  } else if (motionState == LOCKED) {
    greenPattern = LED_PATTERN_SOLID;
  }

  digitalWrite(LED_RED_PIN, ledPatternIsOn(redPattern, nowMs) ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, ledPatternIsOn(greenPattern, nowMs) ? HIGH : LOW);
}

bool ledPatternIsOn(LedPattern pattern, uint32_t nowMs) {
  switch (pattern) {
    case LED_PATTERN_OFF:
      return false;
    case LED_PATTERN_SOLID:
      return true;
    case LED_PATTERN_BLINK_SLOW:
      return ((nowMs / 500U) % 2U) == 0U;
    case LED_PATTERN_BLINK_FAST:
      return ((nowMs / 125U) % 2U) == 0U;
    case LED_PATTERN_DOUBLE_BLINK: {
      const uint32_t phase = nowMs % 1000U;
      return (phase < 100U) || (phase >= 200U && phase < 300U);
    }
  }

  return false;
}

const char *pppStateName(PppState state) {
  switch (state) {
    case SIN_PPP:
      return "SIN_PPP";
    case PPP_CONVERGING:
      return "PPP_CONVERGING";
    case PPP_ESTABLE:
      return "PPP_ESTABLE";
  }

  return "SIN_PPP";
}

const char *motionStateName(MotionState state) {
  switch (state) {
    case MOVING:
      return "MOVING";
    case STOP_CONFIRM:
      return "STOP_CONFIRM";
    case AVERAGING:
      return "AVERAGING";
    case LOCKED:
      return "LOCKED";
  }

  return "MOVING";
}
