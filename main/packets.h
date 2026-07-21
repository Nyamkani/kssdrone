#pragma once

#include <cstdint>

struct LogData
{
    float dt;

    float roll_rad;
    float pitch_rad;

    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;

    float ax_g;
    float ay_g;
    float az_g;

    float throttle_cmd;
    float throttle_used;

    float roll_out;
    float pitch_out;
    float yaw_out;

    float m1;
    float m2;
    float m3;
    float m4;

    uint8_t mode;
    uint8_t state;
};

enum DroneMode : uint8_t
{
    RATE_ACRO = 0,
    ANGLE_SELF_LEVEL = 1
};

enum class LandingMode : uint8_t
{
    NONE = 0,
    SOFT = 1,
    FAILSAFE = 2,
};

enum class DroneState : uint8_t
{
    INIT = 0,
    IMU_BIAS_CALIBRATING = 1,
    DISARMED = 2,
    ARMING = 3,
    ARMED = 4,
    LANDING = 5,
    ERR = 10,
};

enum class PacketType : uint8_t
{
    CONTROL = 0x01,
    TELEMETRY = 0x02,
};


enum ControlCommandFlags : uint8_t
{
    CMD_NONE              = 0,
    CMD_LEVEL_CALIBRATE   = 1 << 0,
    CMD_GYRO_CALIBRATE    = 1 << 1,
    CMD_SOFT_LANDING      = 1 << 2,
    CMD_EMERGENCY_STOP    = 1 << 3,
    CMD_ARM_REQUEST       = 1 << 4,
    CMD_DISARM_REQUEST    = 1 << 5,
    CMD_SET_MODE          = 1 << 6,
};

enum class DisarmReason : uint8_t
{
    NONE = 0,
    CMD_ARM_ZERO = 1,
    COMM_TIMEOUT = 2,
    BATTERY_CRITICAL = 3,
    TILT = 4,
    LANDING_EMERGENCY = 5,
};

struct __attribute__((packed)) TelemetryPacket
{
    PacketType type = PacketType::TELEMETRY; // packet type for future extension

    uint32_t magic = 0xA5A5A5A5; //a fixed magic number to validate the packet
    uint16_t seq = 0; //sequence number for debugging

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;

    float gyro_x_rad_s = 0.0f;
    float gyro_y_rad_s = 0.0f;
    float gyro_z_rad_s = 0.0f;

    float throttle = 0.0f;

    // float pid_out_roll = 0.0f;
    // float pid_out_pitch = 0.0f;
    // float pid_out_yaw = 0.0f;

    // float mix_in_roll = 0.0f;
    // float mix_in_pitch = 0.0f;
    // float mix_in_yaw = 0.0f;

    //test
    // float throttle_cmd = 0.0f;
    // float m1 = 0.0f;
    // float m2 = 0.0f;
    // float m3 = 0.0f;
    // float m4 = 0.0f;

    // float dshot1 = 0.0f;
    // float dshot2 = 0.0f;
    // float dshot3 = 0.0f;
    // float dshot4 = 0.0f;

    uint8_t state = 0;
    uint8_t mode = 0;

    DisarmReason debug_disarm_reason = DisarmReason::NONE;

    uint8_t reserved[2] = {0, 0};
};


struct __attribute__((packed)) ControlPacket
{
    PacketType type = PacketType::CONTROL; // packet type for future extension

    uint32_t magic = 0xA5A5A5A5; //a fixed magic number to validate the packet
    uint16_t seq = 0; //sequence number for debugging
    DroneMode mode = DroneMode::RATE_ACRO;

    float throttle = 0.0f;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rate_rad_s = 0.0f;

    uint8_t cmd_flags = 0;
    uint8_t cmd_seq = 0;
};


