#include "stop_detector.h"
#include "config.h"

namespace {
uint32_t below_threshold_since_ms = 0;
bool latched_stopped = false;
}

namespace stop_detector {

void reset() {
  below_threshold_since_ms = 0;
  latched_stopped = false;
}

bool update(const model::GnssFix& fix, uint32_t now_ms) {
  const bool below_speed = fix.speed_kmh < config::STOP_SPEED_KMH_THRESHOLD;

  if (below_speed) {
    if (below_threshold_since_ms == 0) {
      below_threshold_since_ms = now_ms;
    }
    if ((now_ms - below_threshold_since_ms) >= config::STOP_CONFIRMATION_MS) {
      latched_stopped = true;
    }
  } else {
    below_threshold_since_ms = 0;
    latched_stopped = false;
  }

  return latched_stopped;
}

} // namespace stop_detector
