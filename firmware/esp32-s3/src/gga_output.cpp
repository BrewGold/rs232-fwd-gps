#include "gga_output.h"
#include <Arduino.h>
#include "config.h"
#include "gnss_uart.h"

namespace gga_output {

void begin() {
}

void sendAt10Hz(const model::GnssFix& fix) {
  (void)fix;
}

} // namespace gga_output
