#pragma once

#include <stdint.h>
#include "model.h"

namespace position_averager {

void reset();
void addSample(const model::GnssFix& fix, uint32_t now_ms);
bool isComplete(uint32_t now_ms);
model::GnssFix meanFix();

} // namespace position_averager
