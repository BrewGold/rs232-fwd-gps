#pragma once

#include <stdint.h>
#include "model.h"

namespace stop_detector {

void reset();
bool update(const model::GnssFix& fix, uint32_t now_ms);

} // namespace stop_detector
