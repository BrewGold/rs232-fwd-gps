#include "gga_output.h"
#include <Arduino.h>
#include "config.h"
#include "gnss_uart.h"

namespace {
HardwareSerial dynSerial(config::DYNATEST_UART_NUM);

void sendLine(const String& s) {
  dynSerial.print(s);
  if (!s.endsWith("\r\n")) {
    dynSerial.print("\r\n");
  }
}
} // namespace

namespace gga_output {

void begin() {
  dynSerial.begin(config::DYNATEST_BAUD, SERIAL_8N1, config::PIN_DYNATEST_RX, config::PIN_DYNATEST_TX);
}

void sendAt10Hz(const model::GnssFix& fix) {
  (void)fix;

  const char* gga = gnss_uart::latestGga();
  if (gga && gga[0] != '\0') {
    sendLine(String(gga));
  }
}

} // namespace gga_output
