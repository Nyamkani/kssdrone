#pragma once
// Minimal host-only adapter for definitions normally supplied by the firmware's
// ESP-IDF/include context. No driver or timing behavior is emulated.
using esp_err_t = int;
inline constexpr esp_err_t ESP_OK = 0;
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329577f)
#endif
#include "flight_pid.h"
