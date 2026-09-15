#include "gga_output.h"
#include <Arduino.h>
#include "gnss_uart.h"

namespace {
String preparedGgaLine;
}

namespace gga_output {

void begin() {
  preparedGgaLine.reserve(128);
}

void sendAt10Hz(const model::GnssFix& fix) {
  (void)fix;

  const char* gga = gnss_uart::latestGga();
  if (gga && gga[0] != '\0') {
    preparedGgaLine = gga;
  }
}

const char* latestPreparedGga() {
  return preparedGgaLine.c_str();
}

} // namespace gga_output
