#pragma once

#include <stdint.h>

namespace config {

// UART mapping (ajustar según hardware final)
static constexpr int GNSS_UART_NUM = 1;
static constexpr int DYNATEST_UART_NUM = 2;

static constexpr uint32_t GNSS_BAUD = 115200;
static constexpr uint32_t DYNATEST_BAUD = 38400;

// ESP32-S3 UNO-style pin placeholders (ajustar al cableado real)
static constexpr int PIN_GNSS_RX = 18;
static constexpr int PIN_GNSS_TX = 17;
static constexpr int PIN_DYNATEST_RX = 16;
static constexpr int PIN_DYNATEST_TX = 15;

// Estado de parada
static constexpr float STOP_SPEED_KMH_THRESHOLD = 0.2f;
static constexpr float STOP_DISTANCE_M_THRESHOLD = 0.10f;
static constexpr uint32_t STOP_CONFIRMATION_MS = 2000;

// Promedio
static constexpr uint32_t AVG_WINDOW_MS = 15000;
static constexpr uint32_t OUTPUT_RATE_HZ = 10;
static constexpr uint32_t AVG_SAMPLE_COUNT = AVG_WINDOW_MS / (1000 / OUTPUT_RATE_HZ); // 150

// I2C
static constexpr uint32_t I2C_FREQ_HZ = 100000;

// GPIO placeholders
static constexpr int PIN_LED_POWER = 7;
static constexpr int PIN_LED_GNSS = 8;

} // namespace config
