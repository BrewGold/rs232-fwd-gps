#pragma once

#include <Arduino.h>
#include "model.h"

namespace gnss_uart {

void begin();
bool poll(model::GnssFix& out_fix);
const char* latestGga();

} // namespace gnss_uart
