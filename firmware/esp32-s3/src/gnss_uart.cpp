#include "gnss_uart.h"

namespace gnss_uart {

void begin() {
  // TODO: Inicializar UART1 y parser NMEA (GGA/RMC)
}

bool poll(model::GnssFix& out_fix) {
  (void)out_fix;
  // TODO: Leer y parsear nuevas tramas GNSS
  return false;
}

} // namespace gnss_uart
