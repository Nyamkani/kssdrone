#include "kss_drone.h"

static const char* TAG = "kss_drone";



KSSDrone::KSSDrone() {}
KSSDrone::~KSSDrone() {}

esp_err_t KSSDrone::SetBHandle(board_handles_t bhandle)
{
    this->bhandle_ = bhandle;
    return ESP_OK;
}

esp_err_t KSSDrone::Initialize(board_handles_t bhandle)
{
    if (this->SetBHandle(bhandle) != ESP_OK)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = this->motor_interface_.Initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "motor_interface_.Initialize() failed with error: %d", ret);

        return ret;
    }

    ret = this->pid_controller_.Initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "pid_controller_.Initialize() failed with error: %d", ret);
        return ret;
    }

    ret = this->dt_.Initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "dt_.Initialize() failed with error: %d", ret);
        return ret;
    }

    ret = this->imu_interface_.Initialize(&this->bhandle_);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "imu_interface_.Initialize() failed with error: %d", ret);    
        return ret;
    }

    // ret = this->esp_now_interface_.Initialize();
    // if (ret != ESP_OK)
    // {
    //     return ret;
    // }

    ret = this->crsf_receiver_.Initialize(
        ELRS_UART_NUM,
        GetElrsUartEventQueue()
    );

    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "crsf_receiver_.Initialize() failed with error: %d", ret);
        return ret;
    }

    // ret = smartaudio_.Initialize();

    // if (ret != ESP_OK)
    // {
    //     ESP_LOGI(TAG, "smartaudio_.Initialize() failed with error: %d", ret);
    //     return ret;
    // }

    ret = this->battery_monitor_.Initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "battery_monitor_.Initialize() failed with error: %d", ret);  
        return ret;
    }

    ret = this->led_controller_.Initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "led_controller_.Initialize() failed with error: %d", ret);
        return ret;
    }

    // vTaskDelay(pdMS_TO_TICKS(1000));

    // this->smartaudio_.PrintSettings();

    ret = this->StartTask();
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "StartTask() failed with error: %d", ret);
        return ret;
    }

    return ret;
}



void KSSDrone::BuildAttitudeTarget(const ControlPacket& cmd, AttitudeTarget& target) const
{
    target = {};
    target.throttle = cmd.throttle;
    target.mode = this->drone_mode_;

    float yaw_cmd = cmd.yaw_rate_rad_s;   // normalized stick input

    if (std::fabs(yaw_cmd) < YAW_STICK_DEADBAND_RAD_S)
    {
        yaw_cmd = 0.0f;
    }

    float yaw_rate_cmd =
        -std::clamp(
            yaw_cmd * MAX_YAW_RATE_RAD_S,
            -MAX_YAW_RATE_RAD_S,
             MAX_YAW_RATE_RAD_S
        );

    if (this->drone_mode_ == DroneMode::RATE_ACRO)
    {
        target.roll_rate_rad_s = std::clamp(
            cmd.roll_rad * MAX_ROLL_RATE_RAD_S,
            -MAX_ROLL_RATE_RAD_S,
             MAX_ROLL_RATE_RAD_S
        );

        target.pitch_rate_rad_s = std::clamp(
            cmd.pitch_rad * MAX_PITCH_RATE_RAD_S,
            -MAX_PITCH_RATE_RAD_S,
             MAX_PITCH_RATE_RAD_S
        );

        target.yaw_rate_rad_s = yaw_rate_cmd;

        target.roll_rad = 0.0f;
        target.pitch_rad = 0.0f;
        target.yaw_rad = 0.0f;
    }
    else
    {
        target.roll_rad = std::clamp(
            cmd.roll_rad * MAX_ROLL_ANGLE_RAD,
            -MAX_ROLL_ANGLE_RAD,
             MAX_ROLL_ANGLE_RAD
        );

        target.pitch_rad = std::clamp(
            cmd.pitch_rad * MAX_PITCH_ANGLE_RAD,
            -MAX_PITCH_ANGLE_RAD,
             MAX_PITCH_ANGLE_RAD
        );

        target.yaw_rad = 0.0f;
        target.yaw_rate_rad_s = yaw_rate_cmd;

        target.roll_rate_rad_s = 0.0f;
        target.pitch_rate_rad_s = 0.0f;
    }
}

void KSSDrone::ResetGyroBiasAccumulator()
{
    this->imu_bias_elapsed_dt_ = 0.0f;
    this->gyro_bias_sum_x_ = 0.0f;
    this->gyro_bias_sum_y_ = 0.0f;
    this->gyro_bias_sum_z_ = 0.0f;
    this->gyro_bias_sample_count_ = 0;
}

void KSSDrone::UpdateEkfReady(const float dt)
{
    IMU_PARESED_DATA imu_data{};
    if (this->imu_interface_.GetParsedIMURadsData(imu_data) != ESP_OK)
    {
        this->ekf_ready_ = false;
        this->ekf_ready_dt_ = 0.0f;
        return;
    }

    EKFAttitudeInput ekf_input{};
    ekf_input.dt = dt;
    ekf_input.ax = imu_data.ax_g;
    ekf_input.ay = imu_data.ay_g;
    ekf_input.az = imu_data.az_g;
    ekf_input.gx = imu_data.gx_rad_s;
    ekf_input.gy = imu_data.gy_rad_s;
    ekf_input.gz = imu_data.gz_rad_s;

    const float acc_norm = sqrtf(
        imu_data.ax_g * imu_data.ax_g +
        imu_data.ay_g * imu_data.ay_g +
        imu_data.az_g * imu_data.az_g);

    this->ekf_.Predict(ekf_input);
    if (acc_norm >= IMU_WRONG_DATA_MIN && acc_norm <= IMU_WRONG_DATA_MAX)
    {
        this->ekf_.Update(ekf_input);
    }

    const bool gyro_stable =
        fabs(imu_data.gx_rad_s) < EKF_READY_GYRO_MAX &&
        fabs(imu_data.gy_rad_s) < EKF_READY_GYRO_MAX &&
        fabs(imu_data.gz_rad_s) < EKF_READY_GYRO_MAX;

    const bool accel_ok =
        (acc_norm >= EKF_READY_ACC_MIN) &&
        (acc_norm <= EKF_READY_ACC_MAX);

    if (gyro_stable && accel_ok)
    {
        this->ekf_ready_dt_ += dt;
        if (this->ekf_ready_dt_ >= EKF_READY_TIME_S)
        {
            this->ekf_ready_ = true;
        }
    }
    else
    {
        this->ekf_ready_ = false;
        this->ekf_ready_dt_ = 0.0f;
    }
}

uint16_t KSSDrone::ThrottleToDshotWithIdle(float throttle) const
{
    throttle = std::clamp(throttle, 0.0f, 1.0f);

    if (throttle <= 0.0f)
    {
        return 0;
    }

    if (throttle <= IDLE_THROTTLE)
    {
        return IDLE_DSHOT_THRESHOLD;
    }

    const float norm = (throttle - IDLE_THROTTLE) / (1.0f - IDLE_THROTTLE);
    constexpr uint16_t DSHOT_MAX = 2047;

    return static_cast<uint16_t>(
        IDLE_DSHOT_THRESHOLD +
        norm * static_cast<float>(DSHOT_MAX - IDLE_DSHOT_THRESHOLD));
}

void KSSDrone::UpdateAirborneState(const float dt, const AttitudeTarget& target)
{
    if (this->state_ != DroneState::ARMED && this->state_ != DroneState::LANDING)
    {
        this->is_airborne_ = false;
        this->airborne_confirm_dt_ = 0.0f;
        this->airborne_exit_confirm_dt_ = 0.0f;
        return;
    }

    if (!this->is_airborne_)
    {
        this->airborne_exit_confirm_dt_ = 0.0f;

        if (target.throttle >= AIRBORNE_THROTTLE_ENTER)
        {
            this->airborne_confirm_dt_ += dt;

            if (this->airborne_confirm_dt_ >= AIRBORNE_CONFIRM_TIME)
            {
                this->is_airborne_ = true;
                this->airborne_confirm_dt_ = 0.0f;
            }
        }
        else
        {
            this->airborne_confirm_dt_ = 0.0f;
        }
    }
    else
    {
        this->airborne_confirm_dt_ = 0.0f;

        if (target.throttle <= AIRBORNE_THROTTLE_EXIT)
        {
            this->airborne_exit_confirm_dt_ += dt;

            if (this->airborne_exit_confirm_dt_ >= AIRBORNE_EXIT_CONFIRM_TIME)
            {
                this->is_airborne_ = false;
                this->airborne_exit_confirm_dt_ = 0.0f;
            }
        }
        else
        {
            this->airborne_exit_confirm_dt_ = 0.0f;
        }
    }
}

void KSSDrone::UpdateMainLoopStats(const float raw_dt_s)
{
#if ENABLE_DRONE_MAIN_STATS_LOG
    this->loop_stats_.Update(raw_dt_s);
#else
    (void)raw_dt_s;
#endif
}

void KSSDrone::TryPrintMainLoopStats()
{
#if ENABLE_DRONE_MAIN_STATS_LOG
    if (this->loop_stats_.elapsed_s < 1.0f)
    {
        return;
    }

    ESP_LOGI(TAG,
        "KSS loop state=%d hz=%lu avg=%.3fms min=%.3fms max=%.3fms "
        "over2=%lu over5=%lu over10=%lu wdt=%lu motor=%lu "
        "imu=%lu/%lu/%lu age=%.1f/%lld/%lldus "
        "seq_d=0:%lu 1:%lu 2:%lu 3p:%lu last=%lu/%lu",

        static_cast<int>(this->state_),
        this->loop_stats_.count,
        this->loop_stats_.AvgDtS() * 1000.0f,
        this->loop_stats_.min_dt_s * 1000.0f,
        this->loop_stats_.max_dt_s * 1000.0f,

        this->loop_stats_.over_2ms,
        this->loop_stats_.over_5ms,
        this->loop_stats_.over_10ms,
        this->loop_stats_.wdt_guard,   // 반드시 여기
        static_cast<unsigned long>(this->motor_output_count_),

        this->imu_frame_stats_.frame_count,
        this->imu_frame_stats_.invalid_count,
        this->imu_frame_stats_.stale_count,

        this->imu_frame_stats_.AvgAgeUs(),
        this->imu_frame_stats_.MinAgeUsForLog(),
        this->imu_frame_stats_.age_max_us,

        this->imu_frame_stats_.delta0_count,
        this->imu_frame_stats_.delta1_count,
        this->imu_frame_stats_.delta2_count,
        this->imu_frame_stats_.delta3p_count,

        this->imu_frame_stats_.last_seq_for_log,
        this->imu_frame_stats_.last_sample_count
    );
    this->loop_stats_.Reset();
    this->imu_frame_stats_.ResetKeepLastSeq();
    this->motor_output_count_ = 0;
#endif
}


void KSSDrone::TryPrintArmedProfileStats()
{
#if ENABLE_ARMED_TIMING_PROFILE
    if (this->armed_prof_.report_line == 0 &&
        this->armed_prof_.elapsed_s < ARMED_TIMING_REPORT_SEC)
    {
        return;
    }

    // Test platform: 240MHz fixed CPU frequency, CONFIG_PM_ENABLE=n.
    // Values: average/max in microseconds (sample count).
    const auto avg_us = [this](const ArmedTimingStage stage) -> double
    {
        const auto& p = this->armed_prof_.Get(stage);
        return p.count ? static_cast<double>(p.sum_cycles) /
            (static_cast<double>(p.count) * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) : 0.0;
    };
    const auto max_us = [this](const ArmedTimingStage stage) -> double
    {
        return static_cast<double>(this->armed_prof_.Get(stage).max_cycles) /
            CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    };
    const auto samples = [this](const ArmedTimingStage stage) -> unsigned long
    {
        return static_cast<unsigned long>(this->armed_prof_.Get(stage).count);
    };

    if (this->armed_prof_.report_line == 0)
    {
        ESP_LOGI(TAG,
        "ARMED_TIMING CYCLE us(avg/max,n) work=%.1f/%.1f(%lu) fastbg=%.1f/%.1f(%lu) armed=%.1f/%.1f(%lu) ctx=%.1f/%.1f(%lu) slowbg=%.1f/%.1f(%lu)",
        avg_us(ArmedTimingStage::LOOP_WORK),
        max_us(ArmedTimingStage::LOOP_WORK),
        samples(ArmedTimingStage::LOOP_WORK),
        avg_us(ArmedTimingStage::FAST_BG),
        max_us(ArmedTimingStage::FAST_BG),
        samples(ArmedTimingStage::FAST_BG),
        avg_us(ArmedTimingStage::ARMED),
        max_us(ArmedTimingStage::ARMED),
        samples(ArmedTimingStage::ARMED),
        avg_us(ArmedTimingStage::CONTEXT),
        max_us(ArmedTimingStage::CONTEXT),
        samples(ArmedTimingStage::CONTEXT),
        avg_us(ArmedTimingStage::SLOW_BG),
        max_us(ArmedTimingStage::SLOW_BG),
        samples(ArmedTimingStage::SLOW_BG));
    }


    if (this->armed_prof_.report_line == 1)
    {
        ESP_LOGI(TAG,
        "ARMED_TIMING INPUT us(avg/max,n) fail=%.1f/%.1f(%lu) cmd_run=%.1f/%.1f(%lu) cmd_skip=%.1f/%.1f(%lu) imu=%.1f/%.1f(%lu) est_run=%.1f/%.1f(%lu) est_skip=%.1f/%.1f(%lu)",
        avg_us(ArmedTimingStage::FAILSAFE),
        max_us(ArmedTimingStage::FAILSAFE),
        samples(ArmedTimingStage::FAILSAFE),
        avg_us(ArmedTimingStage::CMD_CALC),
        max_us(ArmedTimingStage::CMD_CALC),
        samples(ArmedTimingStage::CMD_CALC),
        avg_us(ArmedTimingStage::CMD_SKIP),
        max_us(ArmedTimingStage::CMD_SKIP),
        samples(ArmedTimingStage::CMD_SKIP),
        avg_us(ArmedTimingStage::IMU),
        max_us(ArmedTimingStage::IMU),
        samples(ArmedTimingStage::IMU),
        avg_us(ArmedTimingStage::EST_CALC),
        max_us(ArmedTimingStage::EST_CALC),
        samples(ArmedTimingStage::EST_CALC),
        avg_us(ArmedTimingStage::EST_SKIP),
        max_us(ArmedTimingStage::EST_SKIP),
        samples(ArmedTimingStage::EST_SKIP));
    }


    if (this->armed_prof_.report_line == 2)
    {
        ESP_LOGI(TAG,
        "ARMED_TIMING AUX us(avg/max,n) air_run=%.1f/%.1f(%lu) air_skip=%.1f/%.1f(%lu) slow_run=%.1f/%.1f(%lu) slow_skip=%.1f/%.1f(%lu) tilt=%.1f/%.1f(%lu) pid=%.1f/%.1f(%lu)",
        avg_us(ArmedTimingStage::AIR_CALC),
        max_us(ArmedTimingStage::AIR_CALC),
        samples(ArmedTimingStage::AIR_CALC),
        avg_us(ArmedTimingStage::AIR_SKIP),
        max_us(ArmedTimingStage::AIR_SKIP),
        samples(ArmedTimingStage::AIR_SKIP),
        avg_us(ArmedTimingStage::SLOW_CALC),
        max_us(ArmedTimingStage::SLOW_CALC),
        samples(ArmedTimingStage::SLOW_CALC),
        avg_us(ArmedTimingStage::SLOW_SKIP),
        max_us(ArmedTimingStage::SLOW_SKIP),
        samples(ArmedTimingStage::SLOW_SKIP),
        avg_us(ArmedTimingStage::TILT),
        max_us(ArmedTimingStage::TILT),
        samples(ArmedTimingStage::TILT),
        avg_us(ArmedTimingStage::PID),
        max_us(ArmedTimingStage::PID),
        samples(ArmedTimingStage::PID));
    }


    if (this->armed_prof_.report_line == 3)
    {
        ESP_LOGI(TAG,
        "ARMED_TIMING OUTPUT us(avg/max,n) comp_run=%.1f/%.1f(%lu) comp_skip=%.1f/%.1f(%lu) motor=%.1f/%.1f(%lu) rmt=%.1f/%.1f(%lu) m_log=%.1f/%.1f(%lu)",
        avg_us(ArmedTimingStage::OUTC_CALC),
        max_us(ArmedTimingStage::OUTC_CALC),
        samples(ArmedTimingStage::OUTC_CALC),
        avg_us(ArmedTimingStage::OUTC_SKIP),
        max_us(ArmedTimingStage::OUTC_SKIP),
        samples(ArmedTimingStage::OUTC_SKIP),
        avg_us(ArmedTimingStage::MOTOR),
        max_us(ArmedTimingStage::MOTOR),
        samples(ArmedTimingStage::MOTOR),
        avg_us(ArmedTimingStage::MOTOR_RMT),
        max_us(ArmedTimingStage::MOTOR_RMT),
        samples(ArmedTimingStage::MOTOR_RMT),
        avg_us(ArmedTimingStage::MOTOR_LOG),
        max_us(ArmedTimingStage::MOTOR_LOG),
        samples(ArmedTimingStage::MOTOR_LOG));
    }


    if (++this->armed_prof_.report_line >= 4)
    {
        this->armed_prof_.report_line = 0;
        this->armed_prof_.ResetWindow();
    }
#endif
#if ENABLE_IMU_TIMING_PROFILE
    if (this->state_ == DroneState::ARMED &&
        this->imu_timing_.samples >= IMU_TIMING_REPORT_SAMPLES)
    {
        const double denom = static_cast<double>(this->imu_timing_.samples) *
                             static_cast<double>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
        const double mhz = static_cast<double>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
        const auto& p = this->imu_timing_;

        ESP_LOGI(TAG,
                 "IMU_PHASE us(avg/max) total=%.2f/%.2f raw=%.2f/%.2f parse=%.2f/%.2f rest=%.2f/%.2f samples=%lu every=%lu",
                 static_cast<double>(p.sum_cycles) / denom,
                 static_cast<double>(p.max_cycles) / mhz,
                 static_cast<double>(p.raw_sum_cycles) / denom,
                 static_cast<double>(p.raw_max_cycles) / mhz,
                 static_cast<double>(p.parse_sum_cycles) / denom,
                 static_cast<double>(p.parse_max_cycles) / mhz,
                 static_cast<double>(p.remainder_sum_cycles) / denom,
                 static_cast<double>(p.remainder_max_cycles) / mhz,
                 static_cast<unsigned long>(p.samples),
                 static_cast<unsigned long>(IMU_TIMING_SAMPLE_EVERY));

        // Keep the sampling phase while resetting the measurement window.
        const uint32_t sample_slot = this->imu_timing_.sample_slot;
        this->imu_timing_ = {};
        this->imu_timing_.sample_slot = sample_slot;
    }
#endif
}

void KSSDrone::UpdateImuFrameStats(
    const SharedSnapshotFrame<IMU_PARESED_DATA>& frame,
    const int64_t now_us)
{
#if ENABLE_DRONE_MAIN_STATS_LOG
    this->imu_frame_stats_.Update(
        frame.seq,
        frame.data.sample_count,
        frame.timestamp_us,
        now_us,
        frame.valid
    );
#else
    (void)frame;
    (void)now_us;
#endif
}

void KSSDrone::PktDataStore() {}
void KSSDrone::LogDataStore() {}
