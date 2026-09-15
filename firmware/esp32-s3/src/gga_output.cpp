#include "gga_output.h"

namespace gga_output {

void begin() {
  // TODO: Inicializar UART2 a 38400 y salida hacia MAX3232
}

void sendAt10Hz(const model::GnssFix& fix) {
  (void)fix;
  // TODO: Formatear y transmitir GGA a 10 Hz
}

} // namespace gga_output
