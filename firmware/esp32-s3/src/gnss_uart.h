#pragma once

#include <Arduino.h>
#include "model.h"

namespace gnss_uart {

void begin();
bool poll(model::GnssFix& out_fix);
const char* latestGga();
void writeLine(const char* line);
const char* topology();

} // namespace gnss_uart
