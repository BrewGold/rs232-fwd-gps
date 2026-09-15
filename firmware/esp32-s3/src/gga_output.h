#pragma once

#include "model.h"

namespace gga_output {

void begin();
void sendAt10Hz(const model::GnssFix& fix);
const char* latestPreparedGga();

} // namespace gga_output
