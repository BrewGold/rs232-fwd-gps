#include "led_status.h"
#include "config.h"

namespace {
model::GnssQuality current_quality = model::GnssQuality::None;
bool power_on = false;
}

namespace led_status {

void begin() {
  // TODO: Configurar pines LED como salida
}

void setPower(bool on) {
  power_on = on;
  (void)power_on;
  // TODO: Encender/apagar LED_POWER
}

void setGnss(model::GnssQuality quality) {
  current_quality = quality;
  (void)current_quality;
  // TODO: Configurar patrón de parpadeo según calidad
}

void loop() {
  // TODO: Ejecutar máquina de parpadeo no bloqueante
}

} // namespace led_status
