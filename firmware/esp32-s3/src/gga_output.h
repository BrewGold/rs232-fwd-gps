#pragma once

#include "model.h"

namespace gga_output {

void begin();
void sendAt10Hz(const model::GnssFix& fix);

} // namespace gga_output
