#pragma once

#include "model.h"

namespace gnss_uart {

void begin();
bool poll(model::GnssFix& out_fix);

} // namespace gnss_uart
