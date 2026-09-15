#pragma once

namespace model {

enum class GnssQuality {
  None = 0,
  Autonomous,
  SBAS,
  GalileoHAS,
  RTKFix,
};

struct GnssFix {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
  double speed_kmh = 0.0;
  GnssQuality quality = GnssQuality::None;
  bool valid = false;
};

enum class VehicleState {
  Moving = 0,
  Stopped,
  Output,
};

} // namespace model
