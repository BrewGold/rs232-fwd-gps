#include "gnss_uart.h"
#include "config.h"

namespace {
HardwareSerial gnssSerial(config::GNSS_UART_NUM);
String lineBuf;
String latestGgaLine;
constexpr char kActiveTopology[] = "UM980 on TX3/RX3, USB1 debug/logs";

bool parseLatLonNmea(const String& raw, const String& hemi, double& out_deg) {
  if (raw.length() < 3) return false;
  const double v = raw.toDouble();
  if (v == 0.0) return false;

  const int deg = static_cast<int>(v / 100.0);
  const double minutes = v - (deg * 100.0);
  double dec = deg + (minutes / 60.0);

  if (hemi == "S" || hemi == "W") dec = -dec;
  out_deg = dec;
  return true;
}

bool parseGga(const String& sentence, model::GnssFix& fix) {
  if (!sentence.startsWith("$G") || sentence.indexOf("GGA") < 0) return false;

  String fields[20];
  int fieldCount = 0;
  int start = 0;
  for (int i = 0; i <= sentence.length() && fieldCount < 20; ++i) {
    if (i == sentence.length() || sentence[i] == ',') {
      fields[fieldCount++] = sentence.substring(start, i);
      start = i + 1;
    }
  }
  if (fieldCount < 10) return false;

  double lat = 0.0, lon = 0.0;
  if (!parseLatLonNmea(fields[2], fields[3], lat)) return false;
  if (!parseLatLonNmea(fields[4], fields[5], lon)) return false;

  const int quality = fields[6].toInt();
  const double alt = fields[9].toDouble();

  fix.lat_deg = lat;
  fix.lon_deg = lon;
  fix.alt_m = alt;
  fix.valid = true;

  switch (quality) {
    case 4:
      fix.quality = model::GnssQuality::RTKFix;
      break;
    case 2:
      fix.quality = model::GnssQuality::SBAS;
      break;
    case 1:
      fix.quality = model::GnssQuality::Autonomous;
      break;
    default:
      fix.quality = model::GnssQuality::None;
      break;
  }

  latestGgaLine = sentence;
  return true;
}

bool parseRmc(const String& sentence, model::GnssFix& fix) {
  if (!sentence.startsWith("$G") || sentence.indexOf("RMC") < 0) return false;

  String fields[20];
  int fieldCount = 0;
  int start = 0;
  for (int i = 0; i <= sentence.length() && fieldCount < 20; ++i) {
    if (i == sentence.length() || sentence[i] == ',') {
      fields[fieldCount++] = sentence.substring(start, i);
      start = i + 1;
    }
  }
  if (fieldCount < 8) return false;

  const double speed_knots = fields[7].toDouble();
  fix.speed_kmh = speed_knots * 1.852;
  return true;
}
} // namespace

namespace gnss_uart {

void begin() {
  gnssSerial.begin(config::GNSS_BAUD, SERIAL_8N1, config::PIN_GNSS_RX, config::PIN_GNSS_TX);
  lineBuf.reserve(128);
  latestGgaLine.reserve(128);
}

bool poll(model::GnssFix& out_fix) {
  bool updated = false;

  while (gnssSerial.available() > 0) {
    const char c = static_cast<char>(gnssSerial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineBuf.length() > 6) {
        String sentence = lineBuf;
        lineBuf = "";

        if (parseGga(sentence, out_fix)) {
          updated = true;
        }
        parseRmc(sentence, out_fix);
      } else {
        lineBuf = "";
      }
      continue;
    }

    if (lineBuf.length() < 180) {
      lineBuf += c;
    }
  }

  return updated;
}

const char* latestGga() {
  return latestGgaLine.c_str();
}

void writeLine(const char* line) {
  if (!line || line[0] == '\0') return;

  gnssSerial.print(line);
  if (!String(line).endsWith("\r\n")) {
    gnssSerial.print("\r\n");
  }
}

const char* topology() {
  return kActiveTopology;
}

} // namespace gnss_uart
