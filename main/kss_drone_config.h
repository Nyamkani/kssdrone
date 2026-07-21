#pragma once

#include <cstdint>

#define MAIN_LOOP_RATE_HZ 2000

static_assert(
    MAIN_LOOP_RATE_HZ == 1000 ||
    MAIN_LOOP_RATE_HZ == 2000 ||
    MAIN_LOOP_RATE_HZ == 4000,
    "Unsupported MAIN_LOOP_RATE_HZ"
);

static constexpr float MAIN_LOOP_DT_SEC =
    1.0f / static_cast<float>(MAIN_LOOP_RATE_HZ);

static constexpr int64_t MAIN_LOOP_PERIOD_US =
    1000000LL / MAIN_LOOP_RATE_HZ;

static constexpr float MAIN_LOOP_DT_MIN_SEC =
    MAIN_LOOP_DT_SEC * 0.5f;

static constexpr float MAIN_LOOP_DT_MAX_SEC =
    0.005f;

static constexpr float MAIN_LOOP_WDT_GUARD_DT_SEC =
    MAIN_LOOP_DT_SEC * 1.10f;

static constexpr uint32_t MAIN_LOOP_WDT_GUARD_SCORE_LIMIT =
    static_cast<uint32_t>(MAIN_LOOP_RATE_HZ * 0.060f);

static constexpr float MAIN_LOOP_WDT_GUARD_TIME_SEC =
    0.500f;

static constexpr float MAIN_LOOP_FORCE_IDLE_INTERVAL_SEC =
    0.100f;


/*
 * Armed internal rates
 */
#define ARMED_FAILSAFE_RATE_HZ       500
#define ARMED_COMMAND_RATE_HZ        200
#define ARMED_ESTIMATOR_RATE_HZ      500
#define ARMED_TILT_SAFETY_RATE_HZ    500
#define ARMED_AIRBORNE_RATE_HZ       250
#define ARMED_SOFT_COMP_RATE_HZ      250
#define ARMED_OUTPUT_COMP_RATE_HZ    100

#define ARMED_CONTROL_RATE_HZ        2000

#define ARMED_MOTOR_OUTPUT_RATE_HZ   1000
/*
2kHz:
  IMU
  PID

1kHz:
  DShot

500Hz:
  failsafe
  EKF
  tilt safety

250Hz:
  command
  airborne
  soft comp

125Hz:
  output comp
*/


static_assert(
    MAIN_LOOP_RATE_HZ % ARMED_COMMAND_RATE_HZ == 0,
    "MAIN_LOOP_RATE_HZ must be divisible by ARMED_COMMAND_RATE_HZ"
);

static_assert(
    MAIN_LOOP_RATE_HZ % ARMED_ESTIMATOR_RATE_HZ == 0,
    "MAIN_LOOP_RATE_HZ must be divisible by ARMED_ESTIMATOR_RATE_HZ"
);

static_assert(
    MAIN_LOOP_RATE_HZ % ARMED_AIRBORNE_RATE_HZ == 0,
    "MAIN_LOOP_RATE_HZ must be divisible by ARMED_AIRBORNE_RATE_HZ"
);

static_assert(
    MAIN_LOOP_RATE_HZ % ARMED_SOFT_COMP_RATE_HZ == 0,
    "MAIN_LOOP_RATE_HZ must be divisible by ARMED_SOFT_COMP_RATE_HZ"
);

static constexpr float ARMED_COMMAND_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_COMMAND_RATE_HZ);

static constexpr float ARMED_ESTIMATOR_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_ESTIMATOR_RATE_HZ);

static constexpr float ARMED_FAILSAFE_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_FAILSAFE_RATE_HZ);

static constexpr float ARMED_TILT_SAFETY_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_TILT_SAFETY_RATE_HZ);

static constexpr float ARMED_AIRBORNE_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_AIRBORNE_RATE_HZ);

static constexpr float ARMED_SOFT_COMP_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_SOFT_COMP_RATE_HZ);

static constexpr float ARMED_OUTPUT_COMP_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_OUTPUT_COMP_RATE_HZ);

static constexpr float ARMED_CONTROL_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_CONTROL_RATE_HZ);

static constexpr float ARMED_MOTOR_OUTPUT_PERIOD_SEC =
    1.0f / static_cast<float>(ARMED_MOTOR_OUTPUT_RATE_HZ);

/*
 * IMU interrupt_latest용 divider.
 * direct read 모드에서는 사용하지 않아도 된다.
 */
#define IMU_SENSOR_DRDY_HZ 8000

static_assert(
    IMU_SENSOR_DRDY_HZ % MAIN_LOOP_RATE_HZ == 0,
    "IMU_SENSOR_DRDY_HZ must be divisible by MAIN_LOOP_RATE_HZ"
);

#define IMU_INTERRUPT_READ_DIV \
    (IMU_SENSOR_DRDY_HZ / MAIN_LOOP_RATE_HZ)

