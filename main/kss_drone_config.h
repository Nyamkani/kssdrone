#pragma once

#include <cstdint>

#define MAIN_LOOP_RATE_HZ 2000

static_assert(
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

// The 4kHz profile is an experimental 4kHz scheduler tick; IMU/PID
// remain 2kHz, not 4kHz, because the measured IMU+PID cost is ~234us.
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

// Fixed 20ms cyclic executive for ARMED, independent of per-job dt polling.
// These are NOMINAL sampled averages from a profiling-enabled build, NOT WCET.
// Safety/FSM state transitions are deliberately outside this scheduling table.
// 2kHz: 40 slots x 500us. 4kHz: 80 slots x 250us (experimental).
namespace KssArmedSchedule
{
enum class Job : uint8_t
{
    FAILSAFE, COMMAND, IMU, ESTIMATOR, AIRBORNE,
    SLOW_COMP, TILT, PID, OUTPUT_COMP, MOTOR, COUNT
};

constexpr uint16_t Bit(const Job job)
{
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(job));
}

static constexpr uint16_t kSlotCount = MAIN_LOOP_RATE_HZ / 50U;

constexpr uint16_t Mask(const uint16_t slot)
{
    uint16_t m = 0U;
#if MAIN_LOOP_RATE_HZ == 2000
    // IMU and PID: each 500us slot; motor on even slots, EKF on phase 1.
    m |= Bit(Job::IMU) | Bit(Job::PID);
    if (slot % 2U == 0U) m |= Bit(Job::MOTOR);
    if (slot % 4U == 1U) m |= Bit(Job::ESTIMATOR) | Bit(Job::TILT);
    if (slot % 4U == 3U) m |= Bit(Job::FAILSAFE);
    if (slot % 8U == 3U) m |= Bit(Job::SLOW_COMP);
    if (slot % 8U == 7U) m |= Bit(Job::AIRBORNE);
    if (slot % 20U == 7U) m |= Bit(Job::OUTPUT_COMP);
    // Four command executions / 20ms: 4ms/6ms intervals, rather than
    // forcing a 5ms command onto a motor or EKF-heavy slot.
    if (slot == 3U || slot == 15U || slot == 23U || slot == 35U)
        m |= Bit(Job::COMMAND);
#elif MAIN_LOOP_RATE_HZ == 4000
    // EXPERIMENT: 4kHz tick; IMU/PID 2kHz, motor 1kHz, EKF 500Hz.
    // The sampled ~259us IMU+PID+loop overhead already exceeds 250us.
    if (slot % 2U == 0U) m |= Bit(Job::IMU) | Bit(Job::PID);
    if (slot % 8U == 1U || slot % 8U == 5U) m |= Bit(Job::MOTOR);
    if (slot % 8U == 3U) m |= Bit(Job::ESTIMATOR);
    if (slot % 8U == 7U) m |= Bit(Job::FAILSAFE) | Bit(Job::TILT);
    if (slot % 16U == 7U) m |= Bit(Job::SLOW_COMP);
    if (slot % 16U == 15U) m |= Bit(Job::AIRBORNE);
    if (slot % 40U == 15U) m |= Bit(Job::OUTPUT_COMP);
    if (slot == 7U || slot == 23U || slot == 47U || slot == 63U)
        m |= Bit(Job::COMMAND);
#endif
    return m;
}

constexpr uint16_t Count(const Job job)
{
    uint16_t n = 0U;
    for (uint16_t slot = 0U; slot < kSlotCount; ++slot)
        if ((Mask(slot) & Bit(job)) != 0U) ++n;
    return n;
}

// Predictive capacity estimate, not guaranteed on hardware.
constexpr uint16_t EstimateUs(const uint16_t mask)
{
    uint16_t us = 25U;  // ~18us fast-bg + ~7us context (profiled)
    if (mask & Bit(Job::IMU))         us += 138U;
    if (mask & Bit(Job::PID))         us += 96U;
    if (mask & Bit(Job::MOTOR))       us += 182U;
    if (mask & Bit(Job::ESTIMATOR))   us += 215U;
    if (mask & Bit(Job::COMMAND))     us += 80U;
    if (mask & Bit(Job::FAILSAFE))    us += 10U;
    if (mask & Bit(Job::AIRBORNE))    us += 14U;
    if (mask & Bit(Job::SLOW_COMP))   us += 111U;
    if (mask & Bit(Job::TILT))        us += 10U;
    if (mask & Bit(Job::OUTPUT_COMP)) us += 45U;
    return us;
}
constexpr uint16_t EstimatedPeakUs()
{
    uint16_t peak = 0U;
    for (uint16_t slot = 0U; slot < kSlotCount; ++slot)
    {
        const uint16_t cost = EstimateUs(Mask(slot));
        if (cost > peak) peak = cost;
    }
    return peak;
}
static_assert(ARMED_CONTROL_RATE_HZ == 2000 &&
              ARMED_MOTOR_OUTPUT_RATE_HZ == 1000 &&
              ARMED_ESTIMATOR_RATE_HZ == 500 &&
              ARMED_COMMAND_RATE_HZ == 200 &&
              ARMED_SOFT_COMP_RATE_HZ == 250 &&
              ARMED_OUTPUT_COMP_RATE_HZ == 100,
              "Fixed schedule phases must be updated with subrate constants");
static_assert(Count(Job::IMU) == 40U && Count(Job::PID) == 40U &&
              Count(Job::MOTOR) == 20U && Count(Job::ESTIMATOR) == 10U &&
              Count(Job::COMMAND) == 4U && Count(Job::FAILSAFE) == 10U &&
              Count(Job::AIRBORNE) == 5U && Count(Job::SLOW_COMP) == 5U &&
              Count(Job::TILT) == 10U && Count(Job::OUTPUT_COMP) == 2U,
              "Fixed schedule does not meet the intended job counts");
#if MAIN_LOOP_RATE_HZ == 2000
static_assert(EstimatedPeakUs() <= MAIN_LOOP_PERIOD_US,
              "Even the 2kHz estimated average-cost plan is infeasible");
#endif
} // namespace KssArmedSchedule

/*
 * IMU interrupt_latest용 divider.
 * direct read 모드에서는 사용하지 않아도 된다.
 */
// ICM42688P configured ODR=8kHz; previously observed UI_DRDY IRQ about 4kHz.
// Use observed IRQ rate for interrupt-mode divider, not the register ODR.
#define IMU_SENSOR_DRDY_HZ 8000

static_assert(
    IMU_SENSOR_DRDY_HZ % MAIN_LOOP_RATE_HZ == 0,
    "IMU_SENSOR_DRDY_HZ must be divisible by MAIN_LOOP_RATE_HZ"
);

#define IMU_INTERRUPT_READ_DIV \
    (IMU_SENSOR_DRDY_HZ / MAIN_LOOP_RATE_HZ)

