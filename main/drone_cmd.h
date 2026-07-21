#pragma once

#include <cstdint>

enum ControlCommandFlags : uint8_t
{
    CMD_NONE              = 0,
    CMD_LEVEL_CALIBRATE   = 1 << 0,
    CMD_GYRO_CALIBRATE    = 1 << 1,
    CMD_EMERGENCY_STOP    = 1 << 2,
};
