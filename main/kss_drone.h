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
#define FW_VERSION    0.5f

//input scaling
#define MAX_ROLL_RATE_RAD_S   3.0f    // 약 172 deg/s
#define MAX_PITCH_RATE_RAD_S  3.0f
#define MAX_YAW_RATE_RAD_S    2.0f    // yaw는 조금 낮게 시작 추천

#define MAX_ROLL_ANGLE_RAD    DEG2RAD(20.0f)
#define MAX_PITCH_ANGLE_RAD   DEG2RAD(20.0f)

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
#define TILT_LIMIT_RAD (70.0f * 3.1415926f / 180.0f)  //1.221rad 70deg
#define TILT_TRIGGER_DT   0.3 //s

//normalize accel
#define IMU_WRONG_DATA_MIN 0.7f
#define IMU_WRONG_DATA_MAX 1.3f

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

struct ArmedContext
{
    const ControlPacket* cmd = nullptr;

    AttitudeTarget target{};
    IMU_PARESED_DATA imu{};
    EKFAttitudeInput ekf_input{};
    EKFAttitudeOutput ekf_out{};
    AttitudeState state{};

    FlightPIDOutput pid_out{};
    MixerInput mix_in{};
    MixerOutput mix_out{};

    float throttle_rate_cmd = 0.0f;
    float thrust_cmd = 0.0f;

    float final_axis_rp_scale = 1.0f;
    float final_axis_yaw_scale = 1.0f;
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


        float armed_slow_comp_dt_accum_ = 0.0f;

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

        esp_err_t ArmedPrepareFast(const float dt, ArmedContext& ctx);
        esp_err_t ArmedControlFast(const float dt, ArmedContext& ctx);
        esp_err_t ArmedOutputFast(const float dt, ArmedContext& ctx);
        void ArmedSlowCompUpdate(const float dt, ArmedContext& ctx);
        void UpdatePseudoVelocitySlow(const float dt, ArmedContext& ctx);

        void UpdateVoltageCompSlow(ArmedContext& ctx);
        void UpdateAutoTrimSlow(const float dt, ArmedContext& ctx);

        //auto bias
        esp_err_t ImuBiasCalibrating(const float dt);

        void ResetGyroBiasAccumulator();

        //imu log
        void ResetImuFrameStats();
        void UpdateImuFrameStats(const SharedSnapshotFrame<IMU_PARESED_DATA>& frame, int64_t now_us);

        //log
        void UpdateMainLoopStats(float raw_dt_s);
        void TryPrintMainLoopStats();

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