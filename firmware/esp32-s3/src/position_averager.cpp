#include "position_averager.h"
#include "config.h"

namespace {
uint32_t window_start_ms = 0;
uint32_t sample_count = 0;
double sum_lat = 0.0;
double sum_lon = 0.0;
double sum_alt = 0.0;
model::GnssQuality best_quality = model::GnssQuality::None;
}

namespace position_averager {

void reset() {
  window_start_ms = 0;
  sample_count = 0;
  sum_lat = sum_lon = sum_alt = 0.0;
  best_quality = model::GnssQuality::None;
}

void addSample(const model::GnssFix& fix, uint32_t now_ms) {
  if (!fix.valid) return;
  if (window_start_ms == 0) window_start_ms = now_ms;

  sum_lat += fix.lat_deg;
  sum_lon += fix.lon_deg;
  sum_alt += fix.alt_m;
  sample_count++;

  if (static_cast<int>(fix.quality) > static_cast<int>(best_quality)) {
    best_quality = fix.quality;
  }
}

bool isComplete(uint32_t now_ms) {
  if (window_start_ms == 0) return false;
  if (sample_count >= config::AVG_SAMPLE_COUNT) return true;
  return (now_ms - window_start_ms) >= config::AVG_WINDOW_MS;
}

model::GnssFix meanFix() {
  model::GnssFix out;
  if (sample_count == 0) return out;

  out.valid = true;
  out.lat_deg = sum_lat / static_cast<double>(sample_count);
  out.lon_deg = sum_lon / static_cast<double>(sample_count);
  out.alt_m = sum_alt / static_cast<double>(sample_count);
  out.quality = best_quality;
  return out;
}

} // namespace position_averager
