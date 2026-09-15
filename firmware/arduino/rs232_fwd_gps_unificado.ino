#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ======================================================
//  RS232 FWD GPS - UNIFICADO (.ino)
//  - Lee GNSS UM980 por UART_IN
//  - Lee heading de magnetómetro I2C (QMC5883L típico)
//  - Calcula coordenada corregida por offset
//  - Genera y ENVÍA $GPGGA (GPSGGA) por UART_OUT -> MAX3232
//  - Logs por USB (Serial)
// ======================================================

// ================== CONFIGURACIÓN =====================
// --- UART entrada GNSS (UM980 TX3/RX3 hacia ESP32) ---
static const int GNSS_RX_PIN   = 44;      // ESP32 RX <- TX3 UM980
static const int GNSS_TX_PIN   = 43;      // ESP32 TX -> RX3 UM980 (opcional)
static const uint32_t GNSS_BAUD = 115200;

// --- UART salida RS232 (ESP32 -> MAX3232) ---
static const int OUT_TX_PIN    = 17;      // ESP32 TX -> RX MAX3232
static const int OUT_RX_PIN    = 18;      // ESP32 RX <- TX MAX3232 (opcional)
static const uint32_t OUT_BAUD = 115200;  // ajusta a tu receptor si requiere otro

// --- I2C magnetómetro ---
static const int I2C_SDA_PIN = 8;
static const int I2C_SCL_PIN = 9;
static const uint8_t MAG_ADDR = 0x0D;     // QMC5883L típico

// --- LEDs (ajusta a tu placa) ---
static const int LED_GNSS = 2;   // fix
static const int LED_MAG  = 4;   // pulso lectura mag
static const int LED_ERR  = 5;   // error

// --- Navegación/offset ---
static double offsetMeters   = 1.50;   // distancia de corrección
static bool   lateralOffset  = false;  // false=adelante, true=lateral
static bool   offsetToRight  = true;   // si lateral=true
static double declinationDeg = -4.8;   // declinación local (ajustar)
static float  headingAlpha   = 0.20f;  // filtro heading 0..1

// ================== OBJETOS ===========================
HardwareSerial GNSS(1);  // entrada UM980
HardwareSerial OUT(2);   // salida al MAX3232

String nmeaLine;

// Estado GNSS
bool gnssFix = false;
double gnssLat = NAN, gnssLon = NAN, gnssAlt = NAN;
int gnssFixQ = 0;
uint8_t gnssSats = 0;
String gnssUtc = "";

// Estado magnetómetro
bool magOk = false;
float magHeading = NAN;
float headingFiltered = NAN;

// Tiempos/contadores
uint32_t lastLogMs = 0;
uint32_t ggaCount = 0;
uint32_t outCount = 0;

// ================== UTILIDADES ========================
static const double R_EARTH = 6378137.0;

double wrap360(double v) {
  while (v < 0) v += 360.0;
  while (v >= 360.0) v -= 360.0;
  return v;
}

double nmeaToDecimalDegrees(const String& v, const String& hemi) {
  if (v.length() < 3) return NAN;
  double raw = v.toDouble();
  int deg = (int)(raw / 100.0);
  double min = raw - (deg * 100.0);
  double dec = deg + min / 60.0;
  if (hemi == "S" || hemi == "W") dec = -dec;
  return dec;
}

void decimalDegreesToNmea(double deg, bool isLat, String& valueOut, String& hemiOut) {
  double a = fabs(deg);
  int d = (int)a;
  double m = (a - d) * 60.0;

  char buf[24];
  if (isLat) {
    snprintf(buf, sizeof(buf), "%02d%07.4f", d, m); // ddmm.mmmm
    hemiOut = (deg >= 0) ? "N" : "S";
  } else {
    snprintf(buf, sizeof(buf), "%03d%07.4f", d, m); // dddmm.mmmm
    hemiOut = (deg >= 0) ? "E" : "W";
  }
  valueOut = String(buf);
}

void destinationPoint(double latDeg, double lonDeg, double bearingDeg, double distM,
                      double& outLatDeg, double& outLonDeg) {
  double lat1 = latDeg * DEG_TO_RAD;
  double lon1 = lonDeg * DEG_TO_RAD;
  double brng = bearingDeg * DEG_TO_RAD;
  double ang = distM / R_EARTH;

  double sinLat1 = sin(lat1), cosLat1 = cos(lat1);
  double sinAng = sin(ang), cosAng = cos(ang);

  double sinLat2 = sinLat1 * cosAng + cosLat1 * sinAng * cos(brng);
  double lat2 = asin(sinLat2);

  double y = sin(brng) * sinAng * cosLat1;
  double x = cosAng - sinLat1 * sinLat2;
  double lon2 = lon1 + atan2(y, x);

  outLatDeg = lat2 * RAD_TO_DEG;
  outLonDeg = lon2 * RAD_TO_DEG;
}

String nmeaChecksum(const String& sentenceNoDollarNoStar) {
  uint8_t cs = 0;
  for (size_t i = 0; i < sentenceNoDollarNoStar.length(); i++) cs ^= (uint8_t)sentenceNoDollarNoStar[i];
  char b[3];
  snprintf(b, sizeof(b), "%02X", cs);
  return String(b);
}

int splitCSV(const String& s, String out[], int maxFields) {
  int c = 0, st = 0;
  for (int i = 0; i <= s.length(); i++) {
    if (i == s.length() || s[i] == ',') {
      if (c < maxFields) out[c++] = s.substring(st, i);
      st = i + 1;
    }
  }
  return c;
}

bool parseGGA(const String& line, double& lat, double& lon, double& alt, int& fixQ, uint8_t& sats, String& utc) {
  if (!(line.startsWith("$GPGGA") || line.startsWith("$GNGGA"))) return false;

  String core = line;
  int star = core.indexOf('*');
  if (star > 0) core = core.substring(0, star);

  String f[20];
  int n = splitCSV(core, f, 20);
  if (n < 10) return false;

  utc  = f[1];
  lat  = nmeaToDecimalDegrees(f[2], f[3]);
  lon  = nmeaToDecimalDegrees(f[4], f[5]);
  fixQ = f[6].toInt();
  sats = (uint8_t)f[7].toInt();
  alt  = f[9].toDouble();

  if (isnan(lat) || isnan(lon)) return false;
  return true;
}

// Construye SIEMPRE GPSGGA -> "$GPGGA,..."
String buildCorrectedGPGGA(const String& utc, double lat, double lon, int fixQ, uint8_t sats, double altM) {
  String latStr, ns, lonStr, ew;
  decimalDegreesToNmea(lat, true,  latStr, ns);
  decimalDegreesToNmea(lon, false, lonStr, ew);

  String body =
    "GPGGA," + utc + "," + latStr + "," + ns + "," + lonStr + "," + ew + "," +
    String(fixQ) + "," + String(sats) + ",1.0," + String(altM, 2) + ",M,0.0,M,,";

  String cs = nmeaChecksum(body);
  return "$" + body + "*" + cs;
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
  uint8_t got = Wire.requestFrom((int)MAG_ADDR, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

bool initMagnetometerQMC() {
  if (!magWrite8(0x0B, 0x01)) return false; // set/reset period
  if (!magWrite8(0x09, 0x1D)) return false; // OSR=512 RNG=8G ODR=200Hz CONT
  return true;
}

bool readMagHeading(float& headingDegOut) {
  uint8_t raw[6];
  if (!magReadBytes(0x00, raw, 6)) return false;

  int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
  int16_t y = (int16_t)(raw[3] << 8 | raw[2]);

  float hdg = atan2f((float)y, (float)x) * 180.0f / PI;
  if (hdg < 0) hdg += 360.0f;
  headingDegOut = hdg;
  return true;
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

void setFixLeds(bool ok) {
  digitalWrite(LED_GNSS, ok ? HIGH : LOW);
  digitalWrite(LED_ERR,  ok ? LOW  : HIGH);
}

void pulseLed(uint8_t pin, uint16_t ms = 8) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

void runAutoTest10s() {
  Serial.println("\n[autotest] Iniciando prueba de 10s...");
  uint32_t t0 = millis();

  uint32_t ggaIn = 0;
  uint32_t ggaOut = 0;
  uint32_t magReads = 0;
  bool magAnyOk = false;
  bool gnssAnyFix = false;

  String line;
  while (millis() - t0 < 10000) {
    float h;
    if (magOk && readMagHeading(h)) {
      magReads++;
      magAnyOk = true;
      magHeading = h;
      if (isnan(headingFiltered)) headingFiltered = h;
    }

    while (GNSS.available()) {
      char c = (char)GNSS.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          double lat, lon, alt;
          int fixQ;
          uint8_t sats;
          String utc;
          if (parseGGA(line, lat, lon, alt, fixQ, sats, utc)) {
            ggaIn++;
            if (fixQ > 0) gnssAnyFix = true;

            double outLat = lat, outLon = lon;
            if (!isnan(headingFiltered)) {
              double headingTrue = wrap360((double)headingFiltered + declinationDeg);
              double bearing = lateralOffset
                ? wrap360(headingTrue + (offsetToRight ? 90.0 : -90.0))
                : headingTrue;
              destinationPoint(lat, lon, bearing, offsetMeters, outLat, outLon);
            }

            String gpgga = buildCorrectedGPGGA(utc, outLat, outLon, fixQ, sats, alt);
            OUT.println(gpgga);
            ggaOut++;
          }
        }
        line = "";
      } else {
        if (line.length() < 220) line += c;
      }
    }
  }

  Serial.println("[autotest] --- RESULTADOS ---");
  Serial.printf("[autotest] MAG init: %s\n", magOk ? "OK" : "FAIL");
  Serial.printf("[autotest] MAG lecturas: %lu\n", (unsigned long)magReads);
  Serial.printf("[autotest] GNSS GGA IN: %lu\n", (unsigned long)ggaIn);
  Serial.printf("[autotest] GNSS FIX detectado: %s\n", gnssAnyFix ? "SI" : "NO");
  Serial.printf("[autotest] GPGGA OUT (MAX3232): %lu\n", (unsigned long)ggaOut);

  bool pass = (magAnyOk && ggaIn > 0 && ggaOut > 0);
  Serial.printf("[autotest] ESTADO: %s\n\n", pass ? "PASS" : "REVISAR CABLEADO/BAUD/PINES");
}

// ================== SETUP =============================
void setup() {
  Serial.begin(115200);
  delay(300);

  setupLeds();

  GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  OUT.begin(OUT_BAUD, SERIAL_8N1, OUT_RX_PIN, OUT_TX_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  magOk = initMagnetometerQMC();

  Serial.println();
  Serial.println("[boot] .ino unificado listo");
  Serial.printf("[boot] GNSS IN  RX=%d TX=%d @%lu\n", GNSS_RX_PIN, GNSS_TX_PIN, (unsigned long)GNSS_BAUD);
  Serial.printf("[boot] RS232 OUT RX=%d TX=%d @%lu (MAX3232)\n", OUT_RX_PIN, OUT_TX_PIN, (unsigned long)OUT_BAUD);
  Serial.printf("[boot] I2C SDA=%d SCL=%d MAG=0x%02X init=%s\n", I2C_SDA_PIN, I2C_SCL_PIN, MAG_ADDR, magOk ? "OK":"FAIL");
  Serial.printf("[boot] offset=%.2fm lateral=%s right=%s decl=%.2f\n",
                offsetMeters, lateralOffset ? "true":"false", offsetToRight ? "true":"false", declinationDeg);

  runAutoTest10s();
}

// ================== LOOP ==============================
void loop() {
  // 1) Heading magnetómetro
  float h;
  if (magOk && readMagHeading(h)) {
    magHeading = h;

    if (isnan(headingFiltered)) headingFiltered = h;
    else {
      float a = headingFiltered * DEG_TO_RAD;
      float b = h * DEG_TO_RAD;
      float sx = (1.0f - headingAlpha) * cosf(a) + headingAlpha * cosf(b);
      float sy = (1.0f - headingAlpha) * sinf(a) + headingAlpha * sinf(b);
      headingFiltered = atan2f(sy, sx) * 180.0f / PI;
      if (headingFiltered < 0) headingFiltered += 360.0f;
    }
    pulseLed(LED_MAG, 2);
  }

  // 2) Lectura GNSS y salida GPGGA corregida
  while (GNSS.available()) {
    char c = (char)GNSS.read();

    if (c == '\n') {
      nmeaLine.trim();

      if (nmeaLine.length() > 0) {
        double lat, lon, alt;
        int fixQ;
        uint8_t sats;
        String utc;

        if (parseGGA(nmeaLine, lat, lon, alt, fixQ, sats, utc)) {
          gnssFixQ = fixQ;
          gnssFix  = (fixQ > 0);
          gnssLat  = lat;
          gnssLon  = lon;
          gnssAlt  = alt;
          gnssSats = sats;
          gnssUtc  = utc;
          ggaCount++;

          setFixLeds(gnssFix);

          if (gnssFix && !isnan(headingFiltered)) {
            double headingTrue = wrap360((double)headingFiltered + declinationDeg);
            double bearing = headingTrue;

            if (lateralOffset) {
              bearing = wrap360(headingTrue + (offsetToRight ? 90.0 : -90.0));
            }

            double newLat, newLon;
            destinationPoint(gnssLat, gnssLon, bearing, offsetMeters, newLat, newLon);

            // >>> ENVÍO REAL AL MAX3232 COMO GPSGGA ($GPGGA)
            String gpgga = buildCorrectedGPGGA(gnssUtc, newLat, newLon, gnssFixQ, gnssSats, gnssAlt);
            OUT.println(gpgga);
            outCount++;

            uint32_t now = millis();
            if (now - lastLogMs > 500) {
              Serial.printf("RAW lat=%.8f lon=%.8f fix=%d sats=%u\n", gnssLat, gnssLon, gnssFixQ, gnssSats);
              Serial.printf("HDG mag=%.2f filt=%.2f true=%.2f\n", magHeading, headingFiltered, headingTrue);
              Serial.printf("NEW lat=%.8f lon=%.8f | OUT GPSGGA=%lu\n", newLat, newLon, (unsigned long)outCount);
              Serial.println(gpgga);
              lastLogMs = now;
            }
          } else {
            uint32_t now = millis();
            if (now - lastLogMs > 1000) {
              if (!gnssFix) Serial.println("[warn] Sin fix GNSS");
              if (isnan(headingFiltered)) Serial.println("[warn] Sin heading magnetómetro");
              lastLogMs = now;
            }
          }
        }
      }

      nmeaLine = "";
    } else {
      if (nmeaLine.length() < 220) nmeaLine += c;
    }
  }
}
