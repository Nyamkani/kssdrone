#pragma once

#include "imu_interface.h"
#include "flight_pid.h"
#include "motor_interface.h"
#include "motor_mixer.h"
#include "delta_timer.h"
#include "ekf.h"
#include "espnow_interface.h"
#include "adc_battery.h"
#include "led.h"
#include "kss_drone_stats.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "gpio_config.h"
#include "pins.h"

#include <atomic>
#include <cmath>
#include <algorithm>

#include "flight_tune_config.h"
#include "kss_drone_config.h"


/*
state

Init -> IMU_BIAS_CALIBRATING
     -> Disamred
     -> Error

Disamred -> Armming
         -> Error

Arming -> Armed 
       -> Disamred 
       -> Error

Armed -> Disamred
      -> Landing(fail, soft)
      -> Error

Landing -> Disarmed
        -> Error
*/

//structures
/*
1. ControlPacket  -> esp-now protocol
2. AttitudeTarget ->  pid target
3. IMU_PARESED_DATA -> imu data
4. EKFAttitudeInput -> ekf input data
5. EKFAttitudeOutput -> ekf output data
6. FlightPIDOutput -> pid output data
7. MixerOutput -> motor mixer output data
*/

//firmware version
#define FW_VERSION    1.2f

// auto gyro bias calibration
#define GYRO_BIAS_CALIB_TIME_S        1.0f
#define GYRO_BIAS_MAX_ABS_RAD_S       0.15f
#define GYRO_BIAS_ACC_NORM_MIN        0.9f
#define GYRO_BIAS_ACC_NORM_MAX        1.1f

//cut off throttle
#define CUT_OFF_THROTTLE 0.01f
#define IDLE_THROTTLE 0.03f
#define IDLE_DSHOT_THRESHOLD 88

//Tilt trigger
#define TILT_LIMIT_RAD (55.0f * 3.1415926f / 180.0f)  //1.221rad 55deg
#define TILT_TRIGGER_DT   0.3 //s

//normalize accel
#define IMU_WRONG_DATA_MIN 0.98f
#define IMU_WRONG_DATA_MAX 1.02f

#define MS_TO_FREQ(x)    ((uint16_t)(1000.0f/(x)))          // Convert frequency to period in miliseconds
#define FREQ_TO_MS(x)    ((float)(1000.0f/(x)))            // Convert frequency to period in miliseconds

//BackgroundJob 
#define CHECK_BAT_MS  0.05f                // Check battery every 50ms (20Hz)
#define CHECK_SEND_TELEMETRY_MS 0.1f      // Send telemetry every 100ms (10Hz)
#define CHECK_SEND_LOG_MS       0.1f            // Send log everyt 100ms (10hz)

//ekf ready flag
#define EKF_READY_TIME_S        1.0f
#define EKF_READY_GYRO_MAX      0.05f
#define EKF_READY_ACC_MIN       0.9f
#define EKF_READY_ACC_MAX       1.1f

//landing 
#define SOFT_LANDING_RATE      0.05f
#define FAILSAFE_LANDING_RATE  0.05f
#define MAX_LANDING_TIME       30.0f  //30s

//disarmed settle time
#define DISARM_SETTLE_TIME     1.0f

//task log
#define ENABLE_DRONE_MAIN_STATS_LOG  0
#define ENABLE_CMD_DETAIL_LOG 0


//time
#define BACKGROUND_SLOW_JOB_TIME   0.010f
#define ARMED_SLOW_COMP_TIME       0.004f

#define COMMAND_TIMEOUT_US    300000LL

//log
struct ArmedProfileStats
{
    uint64_t count = 0;

    int64_t prepare_us = 0;
    int64_t slow_comp_us = 0;
    int64_t control_us = 0;
    int64_t total_us = 0;

    int64_t prepare_max_us = 0;
    int64_t slow_comp_max_us = 0;
    int64_t control_max_us = 0;
    int64_t total_max_us = 0;
};

struct ArmedContext
{
    /*
     * Command
     */
    const ControlPacket* cmd = nullptr;
    bool cmd_valid = false;
    bool command_updated = false;

    /*
     * Target / setpoint
     */
    AttitudeTarget target{};
    bool target_valid = false;

    /*
     * IMU
     */
    IMU_PARESED_DATA imu{};
    bool imu_valid = false;

    /*
     * EKF
     */
    EKFAttitudeInput ekf_input{};
    EKFAttitudeOutput ekf_out{};
    bool ekf_valid = false;
    bool estimator_updated = false;

    /*
     * Control state
     */
    // 실제 타입명에 맞게 변경
    AttitudeState state{};
    bool state_valid = false;

    /*
     * Control output
     */
    FlightPIDOutput pid_out{};

    /*
     * Extra
     */
    float throttle_rate_cmd = 0.0f;

    bool airborne_updated = false;
};


struct ArmedOutputCache
{
    float yaw_hover_boost = 1.0f;
    float yaw_priority_scale = 1.0f;

    float thrust_v_scale = 1.0f;
    float axis_rp_v_scale = 1.0f;
    float axis_yaw_v_scale = 1.0f;

    float high_thr_scale = 1.0f;
    float ground_blend = 1.0f;

    float fl_mul = 1.0f;
    float fr_mul = 1.0f;
    float rr_mul = 1.0f;
    float rl_mul = 1.0f;
};

class KSSDrone
{
    public:
        KSSDrone();
        virtual ~KSSDrone();


    private:
        IMUInterface imu_interface_;
        EspNowInterface esp_now_interface_;

        FlightPIDController pid_controller_;
        MotorInterface motor_interface_;
        BatteryMonitor battery_monitor_;
        LedController led_controller_;

        QuaternionEKF ekf_;
        DeltaTime dt_;

        TelemetryPacket tpkt_ = {};
        LogData log_= {};

        TaskHandle_t handle_ = nullptr;
        board_handles_t bhandle_ = {};

        DroneState state_ = DroneState::INIT;
        std::atomic<bool> task_stop_{false};

        //Arm, disarm state from remote command
        bool armed_ = false;

        //timer values
        float battery_check_dt_ = 0.0f;
        float arming_check_dt_ = 0.0f;
        float telemetry_send_dt_ = 0.0f;
        float log_send_dt_ = 0.0f;

        //test
        float test_log_dt_ = 0.0f;
        ArmedProfileStats armed_prof_{};

        /*
        * Command / target cache
        */
        bool armed_cmd_valid_ = false;

        AttitudeTarget armed_target_cache_{};
        bool armed_target_valid_ = false;

        float armed_throttle_rate_cmd_ = 0.0f;
        int64_t armed_last_cmd_us_ = 0;

        /*
        * IMU cache
        */
        IMU_PARESED_DATA armed_imu_cache_{};
        bool armed_imu_valid_ = false;

        /*
        * EKF / state cache
        */
        EKFAttitudeInput armed_ekf_input_cache_{};
        EKFAttitudeOutput armed_ekf_out_cache_{};
        bool armed_ekf_valid_ = false;

        // 실제 타입명으로 변경
        AttitudeState armed_state_cache_{};
        bool armed_state_valid_ = false;

        /*
        * Subrate accumulators
        */
        float armed_command_dt_accum_ = 0.0f;
        float armed_estimator_dt_accum_ = 0.0f;
        float armed_airborne_dt_accum_ = 0.0f;
        float armed_slow_comp_dt_accum_ = 0.0f;

        /*
        * output
        */
        ArmedOutputCache armed_output_cache_{};

        float armed_output_comp_dt_accum_ = 0.0f;
        float armed_log_dt_accum_ = 0.0f;

        float armed_control_dt_accum_ = 0.0f;
        float armed_motor_output_dt_accum_ = 0.0f;

        /*
        * failsafe
        */
        float armed_failsafe_dt_accum_ = 0.0f;

        bool armed_pid_valid_ = false;
        FlightPIDOutput armed_pid_out_cache_{};


        float armed_tilt_safety_dt_accum_ = 0.0f;

    private:
        static constexpr float GYRO_LIMIT = 20.0f; // rad/s 

        //imu-lpf
        float gx_lpf_ = 0.0f;
        float gy_lpf_ = 0.0f;
        float gz_lpf_ = 0.0f;

        float ax_lpf_ = 0.0f;
        float ay_lpf_ = 0.0f;
        float az_lpf_ = 0.0f;

        // vertical stabilization
        float vertical_accel_lpf_ = 0.0f;
        float vertical_velocity_est_ = 0.0f;

        //first imu behavior
        bool first_imu_ = true;

        //throttle Ramp
        float throttle_prev_ = 0.0f;

        //check pid controller saturated
        bool output_saturated_ = false;

        //tilt safety check
        float tilt_trigger_dt_ = 0.0f;

        //landing
        LandingMode landing_mode_ = LandingMode::NONE;
        float landing_throttle_ = 0.0f;
        float landing_dt_ = 0.0f;

        // command cache / events
        ControlPacket active_cmd_ = {};
        DroneMode drone_mode_ = DroneMode::RATE_ACRO;
        bool active_cmd_valid_ = false;

        // level offsets
        float roll_offset_rad_  = 0.0000f;
        float pitch_offset_rad_ = 0.0000f;


        // auto trim accum
        float auto_roll_trim_accum_ = 0.0f;
        float auto_pitch_trim_accum_ = 0.0f;

        // auto trim
        float roll_trim_rad_  = 0.000f;     //trim--  -> roll(-)
        float pitch_trim_rad_ = 0.000f;   //trim--  -> pitch(-)

        // designated trim
        float roll_manual_trim_rad_  = DEG2RAD(0.00f); //0.70    //trim--  -> roll(-)
        float pitch_manual_trim_rad_ = DEG2RAD(0.00f); //0.30  //trim--  -> pitch(-)
 
        // auto trim
        float auto_roll_trim_dt_ = 0.0f;
        float auto_pitch_trim_dt_ = 0.0f;

        // dynamic climb trim
        float dynamic_roll_trim_rad_ = 0.0f;
        float dynamic_pitch_trim_rad_ = 0.0f;

        // dynamic takeoff trim
        float takeoff_roll_trim_rad_ = 0.0f;
        float takeoff_pitch_trim_rad_ = 0.0f;

        // mix offsets
        static constexpr float PITCH_HW_TRIM = 0.00f;
        
        //output abs. value 
        //must be reversed from rotation direction
        static constexpr float BASE_ROLL_MIX_OFFSET  =  0.0000f;//0.010f;
        static constexpr float BASE_YAW_MIX_OFFSET   =  0.0000f;

        //air state
        bool is_airborne_ = false;
        float airborne_confirm_dt_ = 0.0f;
        float airborne_exit_confirm_dt_ = 0.0f;

        //disarmed settle dt
        float disarmed_settle_dt_ = 0.0f;

        //550mah
        static constexpr float PITCH_COMP_LOW  = 0.0000f; //0.0230f
        static constexpr float PITCH_COMP_HIGH = 0.0000f; //0.0180f

        static constexpr float ROLL_COMP_LOW   = 0.0000f; //-0.0020f
        static constexpr float ROLL_COMP_HIGH  = 0.0000f; //-0.0010f;

        static constexpr float YAW_COMP_LOW    = 0.0000f;
        static constexpr float YAW_COMP_HIGH   = 0.0000f;//0.0010f + 0.0003f;

        //yaw hold 
        bool yaw_hold_initialized_ = false;
        float yaw_hold_rad_ = 0.0f;

        //slew limit
        float roll_target_smooth_rad_ = 0.0f;
        float pitch_target_smooth_rad_ = 0.0f;
        float yaw_rate_target_smooth_rad_s_ = 0.0f;

        //xyz drift damping
        float ax_world_lpf_ = 0.0f;
        float ay_world_lpf_ = 0.0f;

        float vx_est_ = 0.0f;
        float vy_est_ = 0.0f;

    private:
        // auto gyro bias state
        bool imu_bias_ready_ = false;
        float imu_bias_elapsed_dt_ = 0.0f;

        float gyro_bias_sum_x_ = 0.0f;
        float gyro_bias_sum_y_ = 0.0f;
        float gyro_bias_sum_z_ = 0.0f;
        uint32_t gyro_bias_sample_count_ = 0;

        //efk ready flag
        bool ekf_ready_ = false;
        float ekf_ready_dt_ = 0.0f;
        float landing_yaw_hold_rad_ = 0.0f;

        //cmd flag
        uint8_t last_command_seq_ = 0;
        bool command_seq_ready_ = false;

        uint16_t last_control_seq_ = 0;
        bool control_seq_ready_ = false;

        //task log
        // loop timing statistics
        LoopTimingStats loop_stats_{};
        ImuFrameStats imu_frame_stats_{};
        DisarmReason debug_disarm_reason_ = DisarmReason::NONE;

        float slow_bg_dt_accum_ = 0.0f;

        float cached_pitch_vel_comp_rad_ = 0.0f;
        float cached_roll_vel_comp_rad_ = 0.0f;

        float cached_thrust_v_scale_ = 1.0f;
        float cached_axis_rp_v_scale_ = 1.0f;
        float cached_axis_yaw_v_scale_ = 1.0f;

        //imu
        static constexpr int64_t IMU_STALE_LIMIT_US = 2000;

    public:
        esp_err_t StartTask();
        void StopTaskRequest();
        esp_err_t Initialize(board_handles_t bhandle);

    private:
        static void MainTask(void* param);
        void MainLoop();

        //main jobs (state)
        esp_err_t Init(const float dt);
        esp_err_t Disarmed(const float dt);
        esp_err_t Arming(const float dt);
        esp_err_t Armed(const float dt);
        esp_err_t Landing(const float dt);
        esp_err_t Error(const float dt);
        esp_err_t SetBHandle(board_handles_t bhandle);


        //Jobs
        esp_err_t BackgroundJobs(const float dt);
        esp_err_t FastBackgroundJobs(const float dt);
        esp_err_t SlowBackgroundJobs(const float dt);

        //armed
        void LoadArmedContextCache(ArmedContext& ctx);
        void ResetArmedRuntime();

        esp_err_t ArmedFailsafeFast(float dt);
        esp_err_t ArmedCommandMedium(float dt, ArmedContext& ctx);
        esp_err_t ArmedImuFast(float dt, ArmedContext& ctx);
        esp_err_t ArmedEstimatorMedium(float dt, ArmedContext& ctx);
        void ArmedAirborneSlow(float dt, ArmedContext& ctx);
        void ArmedSlowCompUpdate(float dt, ArmedContext& ctx);
        esp_err_t ArmedControlFast(float dt, ArmedContext& ctx);
        esp_err_t ArmedOutputFast(const float dt, ArmedContext& ctx);
        void ArmedOutputCompMedium(const float dt, ArmedContext& ctx);
        void ArmedTiltSafetyMedium(const float dt, ArmedContext& ctx);


        void UpdatePseudoVelocitySlow(const float dt, ArmedContext& ctx);

        void UpdateVoltageCompSlow(ArmedContext& ctx);
        void UpdateAutoTrimSlow(const float dt, ArmedContext& ctx);

        //auto bias
        esp_err_t ImuBiasCalibrating(const float dt);

        void ResetGyroBiasAccumulator();

        //imu log
        // void ResetImuFrameStats();
        void UpdateImuFrameStats(const SharedSnapshotFrame<IMU_PARESED_DATA>& frame, int64_t now_us);

        //log
        void UpdateMainLoopStats(float raw_dt_s);
        void TryPrintMainLoopStats();
        void TryPrintArmedProfileStats();
        
    private:
        void ChangeState(const DroneState to_state);
        void EnterState(const DroneState to_state);
        void ExitState(const DroneState state);

        void UpdateCommandCache();
        void HandleCommandEvents();

        //Target by mode 
        void BuildAttitudeTarget(const ControlPacket& cmd, AttitudeTarget& target) const;
        
        //ekf ready flag
        void UpdateEkfReady(const float dt);

        //pkt data store
        void PktDataStore();

        //log data store
        void LogDataStore();

        //compensate throttle
        uint16_t ThrottleToDshotWithIdle(float throttle) const;

        //air state
        void UpdateAirborneState(const float dt, const AttitudeTarget& target);

        void UpdateDynamicClimbTrim(const float dt, const AttitudeTarget& target, const float throttle_rate_cmd);

        void UpdateDynamicTakeoffTrim(const AttitudeTarget& target, const float throttle_rate_cmd);

};

inline uint16_t FloatToDshot(float x)
{
    x = std::clamp(x, 0.0f, 1.0f);

    if (x <= 0.0f)
    {
        return 0; // stop
    }

    constexpr uint16_t DSHOT_MIN = 48;
    constexpr uint16_t DSHOT_MAX = 2047;

    return static_cast<uint16_t>(
        DSHOT_MIN + x * static_cast<float>(DSHOT_MAX - DSHOT_MIN) );

}


inline float SlewLimit(float current, float target, float max_rate, float dt)
{
    float delta = target - current;
    float max_delta = max_rate * dt;

    delta = std::clamp(delta, -max_delta, max_delta);
    return current + delta;
}

inline float WrapPi(float x)
{
    while (x > M_PI)  x -= 2.0f * M_PI;
    while (x < -M_PI) x += 2.0f * M_PI;
    return x;
}
inline float SmoothStep01(float x)
{
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// float lerp(float a, float b, float t)
// {
//     return a + (b - a) * t;
// };
static inline int16_t SeqDiff16(uint16_t now, uint16_t prev)
{
    return static_cast<int16_t>(now - prev);
}

static inline void UpdateMaxI64(int64_t& max_value, const int64_t value)
{
    if (value > max_value)
    {
        max_value = value;
    }
}

static inline bool ConsumePeriodicDt(
    float& accum_dt,
    const float dt,
    const float period_sec,
    float& out_dt)
{
    accum_dt += dt;

    if (accum_dt < period_sec)
    {
        return false;
    }

    out_dt = accum_dt;
    accum_dt = 0.0f;

    return true;
}
