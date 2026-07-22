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

    vTaskDelay(pdMS_TO_TICKS(1000));

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
        "over2=%lu over5=%lu over10=%lu wdt=%lu "
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
#endif
}


void KSSDrone::TryPrintArmedProfileStats()
{
#if ENABLE_DRONE_MAIN_STATS_LOG
    constexpr uint64_t PRINT_COUNT = 1000;

    if (this->armed_prof_.count < PRINT_COUNT)
    {
        return;
    }

    const uint64_t count = this->armed_prof_.count;

    ESP_LOGI(TAG,
             "ARMED_PROF avg_us total=%lld prep=%lld slow=%lld ctrl=%lld "
             "max_us total=%lld prep=%lld slow=%lld ctrl=%lld",
             this->armed_prof_.total_us / count,
             this->armed_prof_.prepare_us / count,
             this->armed_prof_.slow_comp_us / count,
             this->armed_prof_.control_us / count,
             this->armed_prof_.total_max_us,
             this->armed_prof_.prepare_max_us,
             this->armed_prof_.slow_comp_max_us,
             this->armed_prof_.control_max_us);

    this->armed_prof_ = {};
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
