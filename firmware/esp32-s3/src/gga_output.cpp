#include "gga_output.h"
#include <Arduino.h>
#include "config.h"
#include "gnss_uart.h"

namespace gga_output {

void begin() {
}

void sendAt10Hz(const model::GnssFix& fix) {
  (void)fix;

  const char* gga = gnss_uart::latestGga();
  if (gga && gga[0] != '\0') {
    gnss_uart::writeLine(gga);
  }
}

} // namespace gga_output
