#pragma once

#include "model.h"

namespace led_status {

void begin();
void setPower(bool on);
void setGnss(model::GnssQuality quality);
void loop();

} // namespace led_status
