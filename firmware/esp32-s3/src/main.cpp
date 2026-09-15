#include <Arduino.h>

#include "config.h"
#include "model.h"
#include "gnss_uart.h"
#include "ntrip_client.h"
#include "stop_detector.h"
#include "position_averager.h"
#include "gga_output.h"
#include "led_status.h"
#include "imu_service.h"

namespace {
model::VehicleState state = model::VehicleState::Moving;
model::GnssFix live_fix;
model::GnssFix output_fix;
uint32_t last_output_ms = 0;
}

void setup() {
  Serial.begin(115200);

  gnss_uart::begin();
  ntrip_client::begin();
  gga_output::begin();
  led_status::begin();
  imu_service::begin();

  stop_detector::reset();
  position_averager::reset();

  led_status::setPower(true);
}

void loop() {
  const uint32_t now_ms = millis();

  ntrip_client::loop();
  imu_service::loop();
  led_status::loop();

  if (gnss_uart::poll(live_fix)) {
    led_status::setGnss(live_fix.quality);

    const bool stopped = stop_detector::update(live_fix, now_ms);

    switch (state) {
      case model::VehicleState::Moving:
        output_fix = live_fix;
        if (stopped) {
          state = model::VehicleState::Stopped;
          position_averager::reset();
        }
        break;

      case model::VehicleState::Stopped:
        position_averager::addSample(live_fix, now_ms);
        if (position_averager::isComplete(now_ms)) {
          output_fix = position_averager::meanFix();
          state = model::VehicleState::Output;
        }
        if (!stopped) {
          state = model::VehicleState::Moving;
          output_fix = live_fix;
        }
        break;

      case model::VehicleState::Output:
        if (!stopped) {
          state = model::VehicleState::Moving;
          output_fix = live_fix;
        }
        break;
    }
  }

  const uint32_t output_period_ms = 1000 / config::OUTPUT_RATE_HZ;
  if ((now_ms - last_output_ms) >= output_period_ms) {
    last_output_ms = now_ms;
    if (output_fix.valid) {
      gga_output::sendAt10Hz(output_fix);
    }
  }
}
