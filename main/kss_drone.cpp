#include "kss_drone.h"

static const char* TAG = "kss_drone";

esp_err_t KSSDrone::StartTask()
{
    BaseType_t ret = xTaskCreatePinnedToCore(MainTask, "DroneMainTask", 4096 * 2, this, 6, &(this->handle_), 1);

    if (ret != pdPASS)
    {
        this->handle_ = nullptr;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void KSSDrone::StopTaskRequest()
{
    this->task_stop_.store(true);

    TaskHandle_t handle = this->handle_;
    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}

void KSSDrone::MainTask(void* param)
{
    KSSDrone* this_ = static_cast<KSSDrone*>(param);
    if (this_ == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    this_->MainLoop();
    this_->handle_ = nullptr;
    vTaskDelete(nullptr);
}

void KSSDrone::MainLoop()
{
    const TickType_t period = pdMS_TO_TICKS(1); // 1000 Hz
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        esp_err_t ret = ESP_OK;

        float raw_dt = this->dt_.Update();
        float dt = std::clamp(raw_dt, 0.0005f, 0.005f);

#if ENABLE_DRONE_MAIN_STATS_LOG
        this->loop_dt_min_ = std::min(this->loop_dt_min_, raw_dt);
        this->loop_dt_max_ = std::max(this->loop_dt_max_, raw_dt);
        this->loop_dt_sum_ += raw_dt;
        this->loop_dt_count_++;

        if (raw_dt > 0.002f)
        {
            this->loop_dt_over_2ms_++;
        }

        if (raw_dt > 0.005f)
        {
            this->loop_dt_over_5ms_++;
        }

        if (raw_dt > 0.010f)
        {
            this->loop_dt_over_10ms_++;
        }

        this->loop_dt_log_elapsed_ += raw_dt;

        if (this->loop_dt_log_elapsed_ >= 1.0f)
        {
            const float avg_dt =
                this->loop_dt_sum_ /
                static_cast<float>(this->loop_dt_count_ > 0 ? this->loop_dt_count_ : 1);

            ESP_LOGI(TAG,
                "KSS loop stat state=%d hz=%lu avg=%.3fms min=%.3fms max=%.3fms over2=%lu over5=%lu over10=%lu",
                static_cast<int>(this->state_),
                this->loop_dt_count_,
                avg_dt * 1000.0f,
                this->loop_dt_min_ * 1000.0f,
                this->loop_dt_max_ * 1000.0f,
                this->loop_dt_over_2ms_,
                this->loop_dt_over_5ms_,
                this->loop_dt_over_10ms_
            );

            this->loop_dt_min_ = 999.0f;
            this->loop_dt_max_ = 0.0f;
            this->loop_dt_sum_ = 0.0f;
            this->loop_dt_count_ = 0;
            this->loop_dt_over_2ms_ = 0;
            this->loop_dt_over_5ms_ = 0;
            this->loop_dt_over_10ms_ = 0;
            this->loop_dt_log_elapsed_ = 0.0f;
        }

#endif

        if (this->task_stop_.load())
        {
            break;
        }

        switch (this->state_)
        {
            case DroneState::INIT:
                ret = this->Init(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::IMU_BIAS_CALIBRATING:
                ret = this->ImuBiasCalibrating(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::DISARMED:
                ret = this->Disarmed(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::ARMING:
                ret = this->Arming(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::ARMED:
                ret = this->Armed(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::LANDING:
                ret = this->Landing(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::ERR:
                this->Error(dt);
                break;

            default:
                break;
        }

        this->BackgroundJobs(dt);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t KSSDrone::Init(const float dt)
{
    (void)dt;
    this->ChangeState(DroneState::IMU_BIAS_CALIBRATING);
    ESP_LOGI(TAG, "Init state Done. Move to IMU_BIAS_CALIBRATING state");
    return ESP_OK;
}

esp_err_t KSSDrone::ImuBiasCalibrating(const float dt)
{
    IMU_PARESED_DATA imu_data{};
    esp_err_t ret = this->imu_interface_.GetParsedIMURadsData(imu_data);
    if (ret != ESP_OK)
    {
        return ret;
    }

    const float acc_norm = sqrtf(
        imu_data.ax_g * imu_data.ax_g +
        imu_data.ay_g * imu_data.ay_g +
        imu_data.az_g * imu_data.az_g);

    const bool gyro_still =
        fabs(imu_data.gx_rad_s) < GYRO_BIAS_MAX_ABS_RAD_S &&
        fabs(imu_data.gy_rad_s) < GYRO_BIAS_MAX_ABS_RAD_S &&
        fabs(imu_data.gz_rad_s) < GYRO_BIAS_MAX_ABS_RAD_S;

    const bool accel_ok =
        (acc_norm >= GYRO_BIAS_ACC_NORM_MIN) &&
        (acc_norm <= GYRO_BIAS_ACC_NORM_MAX);

    if (gyro_still && accel_ok)
    {
        this->gyro_bias_sum_x_ += imu_data.gx_rad_s;
        this->gyro_bias_sum_y_ += imu_data.gy_rad_s;
        this->gyro_bias_sum_z_ += imu_data.gz_rad_s;
        this->gyro_bias_sample_count_++;
        this->imu_bias_elapsed_dt_ += dt;
    }
    else
    {
        this->ResetGyroBiasAccumulator();
    }

    if (this->imu_bias_elapsed_dt_ >= GYRO_BIAS_CALIB_TIME_S &&
        this->gyro_bias_sample_count_ > 0)
    {
        IMU_DATA_BIAS bias = {
            IMU_DATA_AX_BIAS,
            IMU_DATA_AY_BIAS,
            IMU_DATA_AZ_BIAS,
            -this->gyro_bias_sum_x_ / static_cast<float>(this->gyro_bias_sample_count_),
            -this->gyro_bias_sum_y_ / static_cast<float>(this->gyro_bias_sample_count_),
            -this->gyro_bias_sum_z_ / static_cast<float>(this->gyro_bias_sample_count_),
            IMU_DATA_TEMP_BIAS};

        this->imu_interface_.IMUSetDataBias(bias);
        this->imu_bias_ready_ = true;

        ESP_LOGI(TAG, "GYRO AUTO BIAS DONE: gx=%.6f gy=%.6f gz=%.6f", bias.gx_b, bias.gy_b, bias.gz_b);
        ESP_LOGI(TAG, "IMU_BIAS_CALIBRATING state Done. Move to Disarmed state");

        this->ChangeState(DroneState::DISARMED);
    }

    return ESP_OK;
}

esp_err_t KSSDrone::Disarmed(const float dt)
{
    this->disarmed_settle_dt_ += dt;

    this->UpdateEkfReady(dt);

    const ControlPacket& cmd = this->active_cmd_;

    if (!this->active_cmd_valid_ || this->esp_now_interface_.IsTimeout())
    {
        return ESP_OK;
    }

    if (this->armed_ &&
        this->ekf_ready_ &&
        this->disarmed_settle_dt_ >= DISARM_SETTLE_TIME &&
        cmd.throttle < 0.05f)
    {
        this->ChangeState(DroneState::ARMING);
        ESP_LOGI(TAG, "Disarmed state done. Move to ARMING state");
    }

    return ESP_OK;
}

esp_err_t KSSDrone::Arming(const float dt)
{
    ControlPacket cmd{};

    if (!this->esp_now_interface_.IsTimeout())
    {
        cmd = this->active_cmd_;
    }
    else
    {
        ESP_LOGW(TAG, "Arming Canncelled due to comm. timeout");
        this->ChangeState(DroneState::DISARMED);
        return ESP_OK;
    }

    if (!this->armed_)
    {
        ESP_LOGW(TAG, "Arming cancelled by DISARM_REQUEST");
        this->ChangeState(DroneState::DISARMED);
        return ESP_OK;
    }

    if (this->arming_check_dt_ <= 2.0f)
    {
        esp_err_t ret = this->motor_interface_.SetMotorOutput(0, 0, 0, 0);
        if (ret != ESP_OK)
        {
            return ret;
        }
        this->arming_check_dt_ += dt;
    }
    else if (this->arming_check_dt_ > 2.0f &&
            cmd.throttle < 0.05f &&
            this->armed_)
    {
        ESP_LOGI(TAG, "Arming state done. Move to ARMED state");
        this->ChangeState(DroneState::ARMED);
    }

    return ESP_OK;
}

esp_err_t KSSDrone::Armed(const float dt)
{
    //1. Check command validity
    // if (!this->active_cmd_valid_)
    // {
        // 아직 timeout은 아니면 마지막 명령으로 제어 유지
        if (this->esp_now_interface_.IsTimeout())
        {
            ESP_LOGE(TAG,
                "Command timeout -> LANDING dt_ms=%lld rx=%lu seq=%lu max_gap_ms=%lld jump=%lu",
                (esp_timer_get_time() - this->esp_now_interface_.GetLastRxTimeUs()) / 1000,
                this->esp_now_interface_.GetRxCount(),
                this->esp_now_interface_.GetLastSeq(),
                this->esp_now_interface_.GetMaxRxGapUs() / 1000,
                this->esp_now_interface_.GetSeqJumpCount()
            );

            this->debug_disarm_reason_ = DisarmReason::COMM_TIMEOUT;
            this->landing_mode_ = LandingMode::FAILSAFE;
            this->ChangeState(DroneState::LANDING);
            return ESP_OK;
        }
    // }

    //2. Cache command for safety checks and control
    const ControlPacket& cmd = this->active_cmd_;
    AttitudeTarget target{};

    if (this->battery_monitor_.GetState() == BatteryState::CRITICAL)
    {
        this->debug_disarm_reason_ = DisarmReason::BATTERY_CRITICAL;
        this->landing_mode_ = LandingMode::FAILSAFE;
        this->ChangeState(DroneState::LANDING);
        ESP_LOGE(TAG, "CRITICAL battery! -> LANDING(FAILSAFE)");
        return ESP_OK;
    }

    if (this->armed_ == false)
    {
        const bool likely_airborne =
            this->is_airborne_ ||
            this->throttle_prev_ > (IDLE_THROTTLE + 0.05f);

        if (likely_airborne)
        {
            // this->debug_disarm_reason_ = DisarmReason::LANDING_EMERGENCY;
            // this->landing_mode_ = LandingMode::SOFT;
            // this->ChangeState(DroneState::LANDING);
            // ESP_LOGW(TAG, "DISARM_REQUEST in ARMED/airborne -> LANDING(SOFT)");
            ESP_LOGI(TAG, "DISARM_REQUEST ignored in ARMED/airborne. Use SOFT_LANDING or EMERGENCY_STOP.");
        }
        else
        {
            this->debug_disarm_reason_ = DisarmReason::CMD_ARM_ZERO;
            this->ChangeState(DroneState::DISARMED);
            ESP_LOGW(TAG, "DISARM_REQUEST on ground -> DISARMED");
        }

        return ESP_OK;
    }

    //3. Build AttitudeTarget structure from command for control and state updates
    this->BuildAttitudeTarget(cmd, target);

    this->roll_target_smooth_rad_ =
        SlewLimit(this->roll_target_smooth_rad_,
                target.roll_rad,
                TARGET_ANGLE_SLEW_RATE,
                dt);

    this->pitch_target_smooth_rad_ =
        SlewLimit(this->pitch_target_smooth_rad_,
                target.pitch_rad,
                TARGET_ANGLE_SLEW_RATE,
                dt);

    this->yaw_rate_target_smooth_rad_s_ =
        SlewLimit(this->yaw_rate_target_smooth_rad_s_,
                target.yaw_rate_rad_s,
                TARGET_YAW_SLEW_RATE,
                dt);


    target.roll_rad = this->roll_target_smooth_rad_;
    target.pitch_rad = this->pitch_target_smooth_rad_;
    target.yaw_rate_rad_s = this->yaw_rate_target_smooth_rad_s_;

    //4. Throttle cut and ramping logic
    float throttle_rate_cmd = 0.0f; //for yaw feed-forward

    const bool throttle_cut =
        (target.throttle <= CUT_OFF_THROTTLE);

    if (throttle_cut)
    {
        target.throttle = 0.0f;
        this->throttle_prev_ = 0.0f;
    }
    else
    {
        float ramp_up_rate = THROTTLE_RAMP_UP_RATE;

        if (!this->is_airborne_)
        {
            ramp_up_rate = TAKEOFF_THROTTLE_RAMP_UP_RATE;
        }

        const float max_up = ramp_up_rate * dt;
        const float max_down = THROTTLE_RAMP_DOWN_RATE * dt;

        float delta = target.throttle - this->throttle_prev_;
        if (delta > 0.0f)
        {
            delta = std::min(delta, max_up);
        }
        else
        {
            delta = std::max(delta, -max_down);
        }

        target.throttle = this->throttle_prev_ + delta;
        this->throttle_prev_ = target.throttle;
        target.throttle = std::max(target.throttle, IDLE_THROTTLE);

        throttle_rate_cmd = delta / dt;

        // if (this->state_ == DroneState::ARMED && target.throttle <= IDLE_THROTTLE + GROUND_BLEND_RANGE)
        // {
        //     this->throttle_prev_ = std::lerp(this->throttle_prev_, IDLE_THROTTLE, dt * 5.0f);
        //     target.throttle = this->throttle_prev_;
        // }
    }

    //5. Get IMU data
    IMU_PARESED_DATA imu_data{};
    esp_err_t ret = this->imu_interface_.GetParsedIMURadsData(imu_data);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (this->first_imu_)
    {
        this->gx_lpf_ = imu_data.gx_rad_s;
        this->gy_lpf_ = imu_data.gy_rad_s;
        this->gz_lpf_ = imu_data.gz_rad_s;
        this->ax_lpf_ = imu_data.ax_g;
        this->ay_lpf_ = imu_data.ay_g;
        this->az_lpf_ = imu_data.az_g;
        this->first_imu_ = false;
    }
    else
    {
        this->gx_lpf_ += LPF_GYRO_ALPHA * (imu_data.gx_rad_s - this->gx_lpf_);
        this->gy_lpf_ += LPF_GYRO_ALPHA * (imu_data.gy_rad_s - this->gy_lpf_);
        this->gz_lpf_ += LPF_GYRO_ALPHA * (imu_data.gz_rad_s - this->gz_lpf_);
        this->ax_lpf_ += LPF_ACC_ALPHA * (imu_data.ax_g - this->ax_lpf_);
        this->ay_lpf_ += LPF_ACC_ALPHA * (imu_data.ay_g - this->ay_lpf_);
        this->az_lpf_ += LPF_ACC_ALPHA * (imu_data.az_g - this->az_lpf_);
    }

    imu_data.gx_rad_s = std::clamp(std::isfinite(this->gx_lpf_) ? this->gx_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu_data.gy_rad_s = std::clamp(std::isfinite(this->gy_lpf_) ? this->gy_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu_data.gz_rad_s = std::clamp(std::isfinite(this->gz_lpf_) ? this->gz_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu_data.ax_g = this->ax_lpf_;
    imu_data.ay_g = this->ay_lpf_;
    imu_data.az_g = this->az_lpf_;

    //6. Update EKF and get attitude state
    EKFAttitudeInput ekf_input{};
    ekf_input.ax = imu_data.ax_g;
    ekf_input.ay = imu_data.ay_g;
    ekf_input.az = imu_data.az_g;
    ekf_input.gx = imu_data.gx_rad_s;
    ekf_input.gy = imu_data.gy_rad_s;
    ekf_input.gz = imu_data.gz_rad_s;
    ekf_input.dt = dt;

    const float acc_norm = sqrtf(
        imu_data.ax_g * imu_data.ax_g +
        imu_data.ay_g * imu_data.ay_g +
        imu_data.az_g * imu_data.az_g);

    this->ekf_.Predict(ekf_input);
    if (acc_norm >= IMU_WRONG_DATA_MIN && acc_norm <= IMU_WRONG_DATA_MAX)
    {
        this->ekf_.Update(ekf_input);
    }

    const EKFAttitudeOutput ekf_out = this->ekf_.GetOutput();

    // const bool yaw_stick_active =
    //     std::fabs(cmd.yaw_rate_rad_s) > YAW_STICK_DEADBAND;

    const bool yaw_stick_active =
        std::fabs(cmd.yaw_rate_rad_s) > YAW_STICK_DEADBAND_RAD_S;

    if (!this->yaw_hold_initialized_)
    {
        this->yaw_hold_rad_ = ekf_out.yaw_rad;
        this->yaw_hold_initialized_ = true;
    }

    if (yaw_stick_active)
    {
        this->yaw_hold_rad_ = ekf_out.yaw_rad;

        target.yaw_hold_enable = false;
    }
    else
    {
        // 스틱 놓으면 마지막 yaw 유지
        target.yaw_rate_rad_s = 0.0f;   // 중요
        target.yaw_hold_enable = true;
    }

    target.yaw_rad = this->yaw_hold_rad_;

    //6.1 climb dynamic trim application
    // this->UpdateDynamicClimbTrim(dt, target, throttle_rate_cmd);

    //6.2 takeoff dynamic trim
    // this->UpdateDynamicTakeoffTrim(target, throttle_rate_cmd);

    AttitudeState state{};
    float roll_control  = ekf_out.roll_rad
                        - this->roll_offset_rad_
                        - this->roll_trim_rad_
                        - this->roll_manual_trim_rad_
                        - this->takeoff_roll_trim_rad_
                        - this->dynamic_roll_trim_rad_;

    float pitch_control = ekf_out.pitch_rad
                        - this->pitch_offset_rad_
                        - this->pitch_trim_rad_
                        - this->pitch_manual_trim_rad_
                        - this->takeoff_pitch_trim_rad_
                        - this->dynamic_pitch_trim_rad_;

    state.roll_rad = roll_control;
    state.pitch_rad = pitch_control;

    state.yaw_rad = ekf_out.yaw_rad;

    state.gyro_x_rad_s = imu_data.gx_rad_s;
    state.gyro_y_rad_s = imu_data.gy_rad_s;
    state.gyro_z_rad_s = imu_data.gz_rad_s;

    //7. Update Airborne state and vertical stabilization
    this->UpdateAirborneState(dt, target);

    //8. Body accel -> world accel + leaky pseudo velocity estimator
    float roll_phys  = ekf_out.roll_rad  - this->roll_offset_rad_;
    float pitch_phys = ekf_out.pitch_rad - this->pitch_offset_rad_;
    float yaw_phys   = ekf_out.yaw_rad;

    float cr = cosf(roll_phys);
    float sr = sinf(roll_phys);

    float cp = cosf(pitch_phys);
    float sp = sinf(pitch_phys);

    float cy = cosf(yaw_phys);
    float sy = sinf(yaw_phys);

    float ax_b = imu_data.ax_g;
    float ay_b = imu_data.ay_g;
    float az_b = imu_data.az_g;

    // body accel -> world accel
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // unit: g
    float ax_world =
        (cy * cp) * ax_b +
        (cy * sp * sr - sy * cr) * ay_b +
        (cy * sp * cr + sy * sr) * az_b;

    float ay_world =
        (sy * cp) * ax_b +
        (sy * sp * sr + cy * cr) * ay_b +
        (sy * sp * cr - cy * sr) * az_b;

    float az_world =
        (-sp) * ax_b +
        (cp * sr) * ay_b +
        (cp * cr) * az_b;

    // remove gravity from z only
    az_world -= 1.0f;

    // LPF
    this->ax_world_lpf_ =
        this->ax_world_lpf_ * (1.0f - AXY_LPF_ALPHA) +
        ax_world * AXY_LPF_ALPHA;

    this->ay_world_lpf_ =
        this->ay_world_lpf_ * (1.0f - AXY_LPF_ALPHA) +
        ay_world * AXY_LPF_ALPHA;

    this->vertical_accel_lpf_ =
        this->vertical_accel_lpf_ * (1.0f - AZ_LPF_ALPHA) +
        az_world * AZ_LPF_ALPHA;

    // integrate pseudo velocity
    this->vx_est_ += this->ax_world_lpf_ * dt;
    this->vy_est_ += this->ay_world_lpf_ * dt;

    this->vertical_velocity_est_ +=
        this->vertical_accel_lpf_ * dt;

    // leakage
    const float leak_xy =
        std::exp(-VXY_LEAK_PER_SEC * dt);

    const float leak_z =
        std::exp(-VZ_LEAK_PER_SEC * dt);

    this->vx_est_ *= leak_xy;
    this->vy_est_ *= leak_xy;
    this->vertical_velocity_est_ *= leak_z;

    // clamp
    this->vx_est_ = std::clamp(this->vx_est_, -1.0f, 1.0f);
    this->vy_est_ = std::clamp(this->vy_est_, -1.0f, 1.0f);

    this->vertical_velocity_est_ =
        std::clamp(this->vertical_velocity_est_,
                -2.0f,
                2.0f);

    //8.1. Horizontal leaky velocity compensation
    const bool rp_stick_active =
        std::fabs(cmd.roll_rad)  > DEG2RAD(3.0f) ||
        std::fabs(cmd.pitch_rad) > DEG2RAD(3.0f);

    if (this->is_airborne_ && !rp_stick_active)
    {
        const float pitch_vel_comp =
            std::clamp(K_VX_TO_PITCH * this->vx_est_,
                    -VXY_COMP_MAX,
                    VXY_COMP_MAX);

        const float roll_vel_comp =
            std::clamp(K_VY_TO_ROLL * this->vy_est_,
                    -VXY_COMP_MAX,
                    VXY_COMP_MAX);

        state.pitch_rad -= pitch_vel_comp;
        state.roll_rad  -= roll_vel_comp;
    }
    else
    {
        this->vx_est_ *= 0.98f;
        this->vy_est_ *= 0.98f;
    }
                
    //9. Check tilt safety and update tilt trigger timer
    const bool tilt_exceeded =
        (fabs(state.roll_rad) > TILT_LIMIT_RAD) ||
        (fabs(state.pitch_rad) > TILT_LIMIT_RAD);

    if (tilt_exceeded)
    {
        this->tilt_trigger_dt_ += dt;
    }
    else
    {
        this->tilt_trigger_dt_ = 0.0f;
    }

    if (this->tilt_trigger_dt_ > TILT_TRIGGER_DT)
    {
        this->debug_disarm_reason_ = DisarmReason::TILT;
        this->landing_mode_ = LandingMode::FAILSAFE;
        this->ChangeState(DroneState::LANDING);
        ESP_LOGE(TAG, "TILT SAFETY TRIGGERED! -> LANDING(FAILSAFE)");
        return ESP_OK;
    }

    //10. PID control
    const bool allow_integrator =
        ((this->state_ == DroneState::ARMED) && this->is_airborne_);

    if (!allow_integrator)
    {
        this->pid_controller_.ResetIntegrator();
    }

    FlightPIDOutput pid_out = this->pid_controller_.Update(target, state, dt, this->output_saturated_);

    //11. Auto trim logic
    //auto trim condition: airborne, low angle, low angular rate, and some throttle to avoid ground effect noise
    const bool allow_auto_roll_trim =
        (this->is_airborne_ &&
        target.mode == DroneMode::ANGLE_SELF_LEVEL &&
        fabs(target.roll_rad) < DEG2RAD(3.0f) &&
        fabs(target.pitch_rad) < DEG2RAD(3.0f) &&
        fabs(state.gyro_x_rad_s) < DEG2RAD(30.0f) &&
        fabs(state.gyro_y_rad_s) < DEG2RAD(30.0f) &&
        fabs(state.gyro_z_rad_s) < DEG2RAD(30.0f) &&
        target.throttle > 0.35f);

    if (allow_auto_roll_trim)
    {
        const float roll_error =
            this->pid_controller_.GetRollAngleLastError();

        this->auto_roll_trim_accum_ += roll_error * dt;
        this->auto_roll_trim_dt_ += dt;

        if (this->auto_roll_trim_dt_ >= AUTO_TRIM_TIME)
        {
            const float avg_roll_error =
                this->auto_roll_trim_accum_ /
                this->auto_roll_trim_dt_;

            float trim_delta =
                -avg_roll_error * AUTO_TRIM_ROLL_GAIN;

            trim_delta = std::clamp(
                trim_delta,
                -DEG2RAD(0.05f),
                DEG2RAD(0.05f)
            );

            this->roll_trim_rad_ += trim_delta;

            this->roll_trim_rad_ =
                std::clamp(this->roll_trim_rad_,
                        -DEG2RAD(3.0f),
                        DEG2RAD(3.0f));

            this->auto_roll_trim_dt_ = 0.0f;
            this->auto_roll_trim_accum_ = 0.0f;
        }
    }
    else
    {
        this->auto_roll_trim_dt_ = 0.0f;
        this->auto_roll_trim_accum_ = 0.0f;
    }

    const bool allow_auto_pitch_trim =
        (this->is_airborne_ &&
        target.mode == DroneMode::ANGLE_SELF_LEVEL &&
        fabs(target.roll_rad) < DEG2RAD(3.0f) &&
        fabs(target.pitch_rad) < DEG2RAD(3.0f) &&
        fabs(state.gyro_x_rad_s) < DEG2RAD(30.0f) &&
        fabs(state.gyro_y_rad_s) < DEG2RAD(30.0f) &&
        fabs(state.gyro_z_rad_s) < DEG2RAD(30.0f) &&
        target.throttle > 0.35f);

    if (allow_auto_pitch_trim)
    {
        const float pitch_error =
            this->pid_controller_.GetPitchAngleLastError();

        this->auto_pitch_trim_accum_ += pitch_error * dt;
        this->auto_pitch_trim_dt_ += dt;

        if (this->auto_pitch_trim_dt_ >= AUTO_TRIM_TIME)
        {
            const float avg_pitch_error =
                this->auto_pitch_trim_accum_ /
                this->auto_pitch_trim_dt_;

            float trim_delta =
                -avg_pitch_error * AUTO_TRIM_PITCH_GAIN;

            trim_delta = std::clamp(
                trim_delta,
                -DEG2RAD(0.05f),
                DEG2RAD(0.05f)
            );

            this->pitch_trim_rad_ += trim_delta;

            this->pitch_trim_rad_ =
                std::clamp(this->pitch_trim_rad_,
                        -DEG2RAD(3.0f),
                        DEG2RAD(3.0f));

            this->auto_pitch_trim_dt_ = 0.0f;
            this->auto_pitch_trim_accum_ = 0.0f;
        }
    }
    else
    {
        this->auto_pitch_trim_dt_ = 0.0f;
        this->auto_pitch_trim_accum_ = 0.0f;
    }


    //12. PID output limiting
    pid_out.throttle = std::clamp(pid_out.throttle, 0.0f, 1.0f);
    pid_out.roll = std::clamp(pid_out.roll, -0.5f, 0.5f);
    pid_out.pitch = std::clamp(pid_out.pitch, -0.5f, 0.5f);
    pid_out.yaw = std::clamp(pid_out.yaw, -0.5f, 0.5f);

    // Low throttle / pre-air authority
    if (!this->is_airborne_)
    {
        const float preair_throttle = std::clamp(
            (target.throttle - IDLE_THROTTLE) /
            (AIRBORNE_THROTTLE_ENTER - IDLE_THROTTLE),
            0.0f,
            1.0f);

        const float preair_scale = 0.80f + 0.20f * preair_throttle;
        const float yaw_preair_scale = std::lerp(0.60f, 1.0f, preair_throttle);

        pid_out.roll  *= preair_scale;
        pid_out.pitch *= preair_scale;
        pid_out.yaw   *= yaw_preair_scale;

        this->vertical_velocity_est_ = 0.0f;
    }

    //13. PID authority reduction at low throttle for better stick centering feel and to prevent ground effect oscillations
    //7. PID authority reduction at low throttle for better stick centering feel and to prevent ground effect oscillations
    const float roll_pitch_comp = PID_MIN_AUTHORITY_RP + (1.0f - PID_MIN_AUTHORITY_RP) * target.throttle;
    const float yaw_comp = 0.85f + 0.15f * target.throttle;
    // const float yaw_comp = PID_MIN_AUTHORITY_YAW + (1.0f - PID_MIN_AUTHORITY_YAW) * target.throttle;

    pid_out.roll *= roll_pitch_comp;
    pid_out.pitch *= roll_pitch_comp;
    pid_out.yaw *= yaw_comp;

    if (target.throttle > TPA_BREAKPOINT)
    {
        float tpa_ratio = (target.throttle - TPA_BREAKPOINT) / (1.0f - TPA_BREAKPOINT);
        tpa_ratio = std::clamp(tpa_ratio, 0.0f, 1.0f);

        float rp_tpa = 1.0f - TPA_RATE_RP * tpa_ratio;
        float yaw_tpa = 1.0f - TPA_RATE_YAW * tpa_ratio;

        rp_tpa = std::clamp(rp_tpa, 0.5f, 1.0f);
        yaw_tpa = std::clamp(yaw_tpa, 0.5f, 1.0f);

        pid_out.roll *= rp_tpa;
        pid_out.pitch *= rp_tpa;
        pid_out.yaw *= yaw_tpa;
    }

    pid_out.roll = std::clamp(pid_out.roll, -0.5f, 0.5f);
    pid_out.pitch = std::clamp(pid_out.pitch, -0.5f, 0.5f);
    pid_out.yaw = std::clamp(pid_out.yaw, -0.5f, 0.5f);


    float rp_activity =
        std::max(std::fabs(pid_out.roll), std::fabs(pid_out.pitch));

    float yaw_protect_blend = std::clamp(
        (rp_activity - YAW_RP_PROTECT_START) /
        (YAW_RP_PROTECT_FULL - YAW_RP_PROTECT_START),
        0.0f,
        1.0f
    );

    yaw_protect_blend = SmoothStep01(yaw_protect_blend);

    float yaw_priority_scale =
        1.0f - YAW_RP_PROTECT_MAX_CUT * yaw_protect_blend;

    yaw_priority_scale = std::clamp(
        yaw_priority_scale,
        YAW_RP_PROTECT_MIN_SCALE,
        1.0f
    );


    // Hover yaw boost
    float throttle_stable = 1.0f - std::clamp(
        std::fabs(throttle_rate_cmd) / YAW_HOVER_THR_RATE_REF,
        0.0f,
        1.0f
    );

    float rp_quiet = 1.0f - std::clamp(
        rp_activity / YAW_HOVER_RP_ACTIVITY_REF,
        0.0f,
        1.0f
    );

    throttle_stable = SmoothStep01(throttle_stable);
    rp_quiet = SmoothStep01(rp_quiet);

    float yaw_hover_boost =
        1.0f + (YAW_HOVER_BOOST_MAX - 1.0f) *
        throttle_stable *
        rp_quiet;

    if (!this->is_airborne_)
    {
        yaw_hover_boost = 1.0f;
    }

    // 1. 조용한 호버에서는 yaw hold 강화
    pid_out.yaw *= yaw_hover_boost;

    // 2. roll/pitch가 바빠지면 yaw 양보
    pid_out.yaw *= yaw_priority_scale;

    // 3. 최종 yaw clamp
    pid_out.yaw = std::clamp(pid_out.yaw, -0.5f, 0.5f);

    // pid_out.roll = 0.0f;
    // pid_out.pitch = 0.0f;
    // pid_out.yaw = 0.0f;

    //14. Thrust curve 
    float t = std::clamp(pid_out.throttle, 0.0f, 1.0f);
    float expo = std::clamp(THRUST_EXPO, 0.0f, 1.0f);

    float thrust_cmd = t * t * (1.0f - expo) + t * expo;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    if (this->state_ == DroneState::ARMED)
    {
        thrust_cmd = std::max(thrust_cmd, IDLE_THROTTLE);
    }

    //15. Voltage compensation
    float voltage = this->battery_monitor_.GetVoltage();

    float v_scale = 1.0f;

    if (voltage > BATTERY_VOLTAGE_MIN_VALID)
    {
        v_scale = BATTERY_VOLTAGE_REF / voltage;
        v_scale = std::clamp(v_scale, 1.0f, V_SCALE_MAX);
    }

    float thrust_v_scale =
        1.0f + (v_scale - 1.0f) * THRUST_VOLT_COMP_GAIN;

    float axis_rp_v_scale =
        1.0f + (v_scale - 1.0f) * AXIS_VOLT_COMP_GAIN_RP;

    float axis_yaw_v_scale =
        1.0f + (v_scale - 1.0f) * AXIS_VOLT_COMP_GAIN_YAW;

    float volt_comp_blend = std::clamp(
        (t - VOLT_COMP_START_THROTTLE) /
        (VOLT_COMP_FULL_THROTTLE - VOLT_COMP_START_THROTTLE),
        0.0f,
        1.0f
    );

    volt_comp_blend = SmoothStep01(volt_comp_blend);

    if (this->is_airborne_)
    {
        volt_comp_blend = 1.0f;
    }

    float final_thrust_v_scale =
        1.0f + (thrust_v_scale - 1.0f) * volt_comp_blend;

    float final_axis_rp_scale =
        1.0f + (axis_rp_v_scale - 1.0f) * volt_comp_blend;

    float final_axis_yaw_scale =
        1.0f + (axis_yaw_v_scale - 1.0f) * volt_comp_blend;

    thrust_cmd *= final_thrust_v_scale;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    //15.1. Vertical velocity damping (not used at landing)
    float vz_damping =
        -this->vertical_velocity_est_ * K_VZ;

    thrust_cmd += vz_damping;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    // 15.2 High throttle output limiting / sag protection
    float high_thr_blend = std::clamp(
        (t - HIGH_THR_LIMIT_START) /
        (HIGH_THR_LIMIT_FULL - HIGH_THR_LIMIT_START),
        0.0f,
        1.0f
    );

    high_thr_blend = SmoothStep01(high_thr_blend);

    float high_thr_scale =
        1.0f - HIGH_THR_MAX_CUT * high_thr_blend;

    thrust_cmd *= high_thr_scale;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);


    //16. Mixer bias
    MixerInput mix_in{};
    mix_in.throttle = thrust_cmd;
    // mix_in.roll = pid_out.roll;
    // mix_in.pitch = pid_out.pitch;
    // mix_in.yaw = pid_out.yaw;
    mix_in.roll  = pid_out.roll * final_axis_rp_scale;
    mix_in.pitch = pid_out.pitch * final_axis_rp_scale;
    mix_in.yaw   = pid_out.yaw * final_axis_yaw_scale;

    // VZ based attitude compensation
    float vz_comp = 0.0f;

    if (this->is_airborne_)
    {
        vz_comp = std::clamp(this->vertical_velocity_est_, -1.0f, 1.0f);
    }

    mix_in.roll  += K_ROLL_VZ_COMP  * vz_comp;
    mix_in.pitch += K_PITCH_VZ_COMP * vz_comp;
    mix_in.yaw   += K_YAW_VZ_COMP   * vz_comp;

    float yaw_throttle_down_ff = 0.0f;

    if (this->is_airborne_ && throttle_rate_cmd < 0.0f)
    {
        yaw_throttle_down_ff =
            -throttle_rate_cmd * YAW_THR_DOWN_FF_GAIN;

        yaw_throttle_down_ff = std::clamp(
            yaw_throttle_down_ff,
            0.0f,
            YAW_THR_DOWN_FF_MAX
        );
    }

    mix_in.yaw += yaw_throttle_down_ff;

    // static yaw bias
    if (this->is_airborne_)
    {
        mix_in.yaw += YAW_MIX_BIAS;
    }
    // float fl_mul = 0.0f, fr_mul = 0.0f, rr_mul = 0.0f, rl_mul = 0.0f;
    // float mixer_throttle = std::clamp(mix_in.throttle, 0.0f, 1.0f);

    // // hover 기준 
    // constexpr float throttle_low  = 0.3f;
    // constexpr float throttle_high = 0.7f;
    // constexpr float hover_throttle = 0.52f;

    // float throttle_blend = std::clamp(
    //     (mixer_throttle - throttle_low) / (throttle_high - throttle_low),
    //     0.0f,
    //     1.0f);
  
    // float front_bias = std::lerp(
    //     FRONT_THRUST_BIAS * 0.97f,   // LOW
    //     FRONT_THRUST_BIAS * 1.03f,   // HIGH
    //     throttle_blend);

    // float rear_bias = std::lerp(
    //     REAR_THRUST_BIAS * 1.03f,    // LOW
    //     REAR_THRUST_BIAS * 0.97f,    // HIGH
    //     throttle_blend);

    float fl_mul = 0.0f, fr_mul = 0.0f, rr_mul = 0.0f, rl_mul = 0.0f;
    float mixer_throttle = std::clamp(mix_in.throttle, 0.0f, 1.0f);

    // hover 기준 
    constexpr float hover_throttle = 0.52f;

    // hover 기준 차이
    float h_dt = mixer_throttle - hover_throttle;

    // 핵심: 부드러운 비선형
    // float k_gain = h_dt * (1.0f + 3.0f * std::abs(h_dt)); // *2.0f;   // 2.0은 gain, 나중에 튜닝
    float k_gain = std::clamp(h_dt, -0.3f, 0.3f);

    // 보정 강도 
    float h_gain = 0.00f; //0.0654;

    // front / rear 반대 방향으로 보정
    //front_bias = 0.914, rear_bias = 1.086, rises backward
    //front_bias = 0.92, rear_bias = 1.08, rises afterward
    //throttle = 0, -> front_bias = 0.95 +(0.08*-0.52) = 0.9084, 
    //                 rear_bias = 1.05 +(0.08*0.52) = 1.0904
    
    float front_bias = 1.0f;
    float rear_bias  = 1.0f;
    float left_bias  = 1.0f;
    float right_bias = 1.0f;

    if (this->is_airborne_)
    {
        front_bias = FRONT_THRUST_BIAS + h_gain * k_gain;
        rear_bias  = REAR_THRUST_BIAS  - h_gain * k_gain;

        left_bias  = LEFT_THRUST_BIAS;
        right_bias = RIGHT_THRUST_BIAS;
    }

    fl_mul = M1_FL_BIAS * front_bias * left_bias;  // FL
    fr_mul = M2_FR_BIAS * front_bias * right_bias; // FR
    rr_mul = M3_RR_BIAS * rear_bias  * right_bias; // RR
    rl_mul = M4_RL_BIAS * rear_bias  * left_bias;  // RL

    //yaw throttle feed-forward
    // float yaw_ff = 0.0f;
    // float yaw_thrust_ff = 0.0015f;
    // if (throttle_rate_cmd  > 0.0f)
    // {
    //     yaw_ff = -throttle_rate_cmd  * yaw_thrust_ff;
    // }

    // yaw_ff = std::clamp(yaw_ff, -0.010f, 0.0f);

    // mix_in.yaw += yaw_ff;

    //18. Mixer with bias and compensation
    MixerOutput mix_out =
        MixMotorsWithMultiplier(mix_in, fl_mul, fr_mul, rr_mul, rl_mul);

    // 18.1. Ground blend
    float ground_blend = 1.0f;

    if (!this->is_airborne_)
    {
        float ground_throttle = std::clamp(
            (target.throttle - IDLE_THROTTLE) / GROUND_BLEND_RANGE,
            0.0f,
            1.0f);

        ground_blend = SmoothStep01(ground_throttle);
        ground_blend = 0.15f + 0.85f * ground_blend;
}

    mix_out.m1 = thrust_cmd + (mix_out.m1 - thrust_cmd) * ground_blend;
    mix_out.m2 = thrust_cmd + (mix_out.m2 - thrust_cmd) * ground_blend;
    mix_out.m3 = thrust_cmd + (mix_out.m3 - thrust_cmd) * ground_blend;
    mix_out.m4 = thrust_cmd + (mix_out.m4 - thrust_cmd) * ground_blend;

    // 18.2. 평균 thrust_cmd로 복귀
    float avg = 0.25f * (mix_out.m1 + mix_out.m2 + mix_out.m3 + mix_out.m4);
    float delta = thrust_cmd - avg;

    mix_out.m1 += delta;
    mix_out.m2 += delta;
    mix_out.m3 += delta;
    mix_out.m4 += delta;

    // 18.3. Yaw output bias도 최종 normalize 전에 적용
    float yaw_output_bias = 0.0f;

    if (this->is_airborne_)
    {
        yaw_output_bias = YAW_OUTPUT_BIAS_AIRBORNE;
    }

    mix_out.m1 += yaw_output_bias;
    mix_out.m2 -= yaw_output_bias;
    mix_out.m3 += yaw_output_bias;
    mix_out.m4 -= yaw_output_bias;

    // 18.4. 최종 normalize는 한 번만
    MixNormalize(mix_out, 0.0f, 1.0f);

    //19. DSHOT conversion and motor output
    uint16_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;

    d1 = ThrottleToDshotWithIdle(mix_out.m1);
    d2 = ThrottleToDshotWithIdle(mix_out.m2);
    d3 = ThrottleToDshotWithIdle(mix_out.m3);
    d4 = ThrottleToDshotWithIdle(mix_out.m4);

    d1 = std::max<uint16_t>(d1, IDLE_DSHOT_THRESHOLD);
    d2 = std::max<uint16_t>(d2, IDLE_DSHOT_THRESHOLD);
    d3 = std::max<uint16_t>(d3, IDLE_DSHOT_THRESHOLD);
    d4 = std::max<uint16_t>(d4, IDLE_DSHOT_THRESHOLD);

    ret = this->motor_interface_.SetMotorOutput(d1, d2, d3, d4);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "RMT - Motor Control failed.");
        return ret;
    }

    //20. Telemetry and logging
    this->tpkt_.roll_rad = state.roll_rad;
    this->tpkt_.pitch_rad = state.pitch_rad;

    this->tpkt_.gyro_x_rad_s = ekf_input.gx;
    this->tpkt_.gyro_y_rad_s = ekf_input.gy;
    this->tpkt_.gyro_z_rad_s = ekf_input.gz;

    this->tpkt_.throttle = target.throttle;

    // this->tpkt_.pid_out_roll = pid_out.roll;
    // this->tpkt_.pid_out_pitch = pid_out.pitch;
    // this->tpkt_.pid_out_yaw = pid_out.yaw;

    // this->tpkt_.mix_in_roll = mix_in.roll;
    // this->tpkt_.mix_in_pitch = mix_in.pitch;
    // this->tpkt_.mix_in_yaw = mix_in.yaw;


    //test
    // this->tpkt_.throttle_cmd  = target.throttle;
    // this->tpkt_.m1 = mix_out.m1;
    // this->tpkt_.m2 = mix_out.m2;
    // this->tpkt_.m3 = mix_out.m3;
    // this->tpkt_.m4 = mix_out.m4;

    // this->tpkt_.dshot1 = d1;
    // this->tpkt_.dshot2 = d2;
    // this->tpkt_.dshot3 = d3;
    // this->tpkt_.dshot4 = d4;

    
    this->tpkt_.debug_disarm_reason = this->debug_disarm_reason_;

    this->log_.dt = dt;
    this->log_.roll_rad = state.roll_rad;
    this->log_.pitch_rad = state.pitch_rad;
    this->log_.gyro_x_rad_s = ekf_input.gx;
    this->log_.gyro_y_rad_s = ekf_input.gy;
    this->log_.gyro_z_rad_s = ekf_input.gz;
    this->log_.ax_g = ekf_input.ax;
    this->log_.ay_g = ekf_input.ay;
    this->log_.az_g = ekf_input.az;
    this->log_.throttle_cmd = cmd.throttle;
    this->log_.throttle_used = target.throttle;
    this->log_.roll_out = pid_out.roll;
    this->log_.pitch_out = pid_out.pitch;
    this->log_.yaw_out = pid_out.yaw;
    this->log_.m1 = mix_out.m1;
    this->log_.m2 = mix_out.m2;
    this->log_.m3 = mix_out.m3;
    this->log_.m4 = mix_out.m4;
    this->log_.mode = static_cast<uint8_t>(this->drone_mode_);
    this->log_.state = static_cast<uint8_t>(this->state_);


#if ENABLE_CMD_DETAIL_LOG
    // this->test_log_dt_ += dt;

    // if(this->test_log_dt_ >= 1.0f)
    // {
    //     while(this->test_log_dt_>= 1.0f)
    //     {
    //         this->test_log_dt_ -=1.0f;
    //     }

    //     // ESP_LOGI(TAG,
    //     //     "CMD state=%d valid=%d timeout=%d rx=%lu seq=%lu dt_ms=%lld cmd arm=%d thr=%.3f r=%.3f p=%.3f y=%.3f target thr=%.3f r=%.3f p=%.3f yr=%.3f d=%u %u %u %u",
    //     //     static_cast<int>(this->state_),
    //     //     this->active_cmd_valid_,
    //     //     this->esp_now_interface_.IsTimeout(),
    //     //     this->esp_now_interface_.GetRxCount(),
    //     //     this->esp_now_interface_.GetLastSeq(),
    //     //     (esp_timer_get_time() - this->esp_now_interface_.GetLastRxTimeUs()) / 1000,
    //     //     cmd.armed,
    //     //     cmd.throttle,
    //     //     cmd.roll_rad,
    //     //     cmd.pitch_rad,
    //     //     cmd.yaw_rate_rad_s,
    //     //     target.throttle,
    //     //     target.roll_rad,
    //     //     target.pitch_rad,
    //     //     target.yaw_rate_rad_s,
    //     //     d1, d2, d3, d4
    //     // );

    //     // ESP_LOGI(TAG, "m1 = %0.3f, m2 = %0.3f, m3 = 0.3f, m4 = 0.4f", 
    //     //                                                     mix_out.m1,
    //     //                                                     mix_out.m2,
    //     //                                                     mix_out.m3,
    //     //                                                     mix_out.m4);

    //     // ESP_LOGI(TAG,
    //     //     "raw ekf r=%.3f p=%.3f y=%.3f | pid_out r=%.3f p=%.3f y=%.3f | state r=%.3f p=%.3f |gx=%.3f gy=%.3f gz=%.3f | m = %.3f %.3f %.3f %.3f",
    //     //     ekf_out.roll_rad,
    //     //     ekf_out.pitch_rad,
    //     //     ekf_out.yaw_rad,
    //     //     pid_out.pitch,
    //     //     pid_out.pitch,
    //     //     pid_out.yaw,
    //     //     state.roll_rad,
    //     //     state.pitch_rad,
    //     //     state.gyro_x_rad_s,
    //     //     state.gyro_y_rad_s,
    //     //     state.gyro_z_rad_s,
    //     //     log_.m1,
    //     //     log_.m2,
    //     //     log_.m3,
    //     //     log_.m4     
    //     //         );

    //     // ESP_LOGI(TAG,
    //     //     " ro=%.3f po=%.3f yo=%.3f | state r=%.3f p=%.3f | gx=%.3f gy=%.3f gz=%.3f | m = %.3f %.3f %.3f %.3f",
    //     //     this->log_.roll_out,
    //     //     this->log_.pitch_out,
    //     //     this->log_.yaw_out,
    //     //     state.roll_rad,
    //     //     state.pitch_rad,
    //     //     state.gyro_x_rad_s,
    //     //     state.gyro_y_rad_s,
    //     //     state.gyro_z_rad_s,
    //     //     log_.m1,
    //     //     log_.m2,
    //     //     log_.m3,
    //     //     log_.m4     
    //     //         );


    //     // ESP_LOGI(TAG,
    //     //     " ro=%.3f po=%.3f yo=%.3f | state r=%.3f p=%.3f | mix_in r=%.3f p=%.3f y=%.3f | m = %.3f %.3f %.3f %.3f",
    //     //     this->log_.roll_out,
    //     //     this->log_.pitch_out,
    //     //     this->log_.yaw_out,
    //     //     state.roll_rad,
    //     //     state.pitch_rad,
    //     //     mix_in.roll,
    //     //     mix_in.pitch,
    //     //     mix_in.yaw,
    //     //     log_.m1,
    //     //     log_.m2,
    //     //     log_.m3,
    //     //     log_.m4     
    //     //         );



    //     // ESP_LOGI(TAG,
    //     //     " ro=%.3f po=%.3f yo=%.3f | state r=%.3f p=%.3f | gx=%.3f gy=%.3f gz=%.3f | ye= %.3f yhr=%.3f yrt=%.3f",
    //     //     this->log_.roll_out,
    //     //     this->log_.pitch_out,
    //     //     this->log_.yaw_out,
    //     //     state.roll_rad,
    //     //     state.pitch_rad,
    //     //     state.gyro_x_rad_s,
    //     //     state.gyro_y_rad_s,
    //     //     state.gyro_z_rad_s,
    //     //     yaw_error,
    //     //     yaw_hold_rate,
    //     //     yaw_rate_target1
    //     //         );

    //     // ESP_LOGI(TAG,
    //     //     "yaw target=%.3f state=%.3f err=%.3f stick=%.3f active=%d hold_en=%d",
    //     //     target.yaw_rad,
    //     //     state.yaw_rad,
    //     //     yaw_error,
    //     //     cmd.yaw_rate_rad_s,
    //     //     yaw_stick_active,
    //     //     target.yaw_hold_enable
    //     // );

    //     // ESP_LOGI(TAG,
    //     //     "gz = %.4f, target.yaw_raw = %.4f, state.yaw_raw= %.4f, yaw pid=%.4f vz=%.4f yaw_vz=%.4f mix_yaw=%.4f m=%.3f %.3f %.3f %.3f",
    //     //     state.gyro_z_rad_s,
    //     //     target.yaw_rad,
    //     //     state.yaw_rad,
    //     //     pid_out.yaw,
    //     //     this->vertical_velocity_est_,
    //     //     K_YAW_VZ_COMP * vz_comp,
    //     //     mix_in.yaw,
    //     //     mix_out.m1,
    //     //     mix_out.m2,
    //     //     mix_out.m3,
    //     //     mix_out.m4
    //     // );


    //     // ESP_LOGI(TAG,
    //     //     "target.pitch_raw = %.4f, state.pitch_raw= %.4f, pitch pid=%.4f mix_pitch=%.4f m=%.3f %.3f %.3f %.3f",
    //     //     target.pitch_rad,
    //     //     state.pitch_rad,
    //     //     pid_out.pitch,
    //     //     mix_in.pitch,
    //     //     mix_out.m1,
    //     //     mix_out.m2,
    //     //     mix_out.m3,
    //     //     mix_out.m4
    //     // );

    //     // ESP_LOGI(TAG,
    //     //     "active=%d hold=%d target=%.3f state=%.3f ye=%.3f yhr=%.3f gz=%.3f ypid=%.3f mix_yaw=%.3f",
    //     //     yaw_stick_active,
    //     //     target.yaw_hold_enable,
    //     //     target.yaw_rad,
    //     //     state.yaw_rad,
    //     //     yaw_error,
    //     //     yaw_hold_rate,
    //     //     state.gyro_z_rad_s,
    //     //     pid_out.yaw,
    //     //     mix_in.yaw
    //     // );

    
    //     // float yaw_error =
    //     //     this->pid_controller_.GetLastYawError();

    //     // float yaw_rate_target =
    //     //     this->pid_controller_.GetLastYawRateTarget();

    //     // float yaw_rate_error =
    //     //     this->pid_controller_.GetYawRateLastError();

    //     // float yaw_i_term =
    //     //     this->pid_controller_.GetYawRateIterm();

    //     // ESP_LOGI(TAG,
    //     //     "thr=%.3f thrust=%.3f yaw_tgt=%.3f yaw=%.3f ye=%.3f yrt=%.3f gz=%.3f yerr=%.3f yi=%.4f ypid=%.4f mix_yaw=%.4f m=%.3f %.3f %.3f %.3f",
    //     //     target.throttle,
    //     //     thrust_cmd,
    //     //     target.yaw_rad,
    //     //     state.yaw_rad,
    //     //     yaw_error,
    //     //     yaw_rate_target,
    //     //     state.gyro_z_rad_s,
    //     //     yaw_rate_error,
    //     //     yaw_i_term,
    //     //     pid_out.yaw,
    //     //     mix_in.yaw,
    //     //     mix_out.m1,
    //     //     mix_out.m2,
    //     //     mix_out.m3,
    //     //     mix_out.m4
    //     // );

    // }
#endif


    
    return ESP_OK;
}

esp_err_t KSSDrone::Landing(const float dt)
{
    // 0. Emergency / disarm check
    ControlPacket cmd{};
    const bool cmd_valid = !this->esp_now_interface_.IsTimeout();

    if (cmd_valid)
    {
        cmd = this->active_cmd_;
    }

    // 사용자가 실제로 disarm 명령을 보낸 경우에만 즉시 정지
    // if (cmd_valid && cmd.armed == 0)
    // {
    //     this->debug_disarm_reason_ = DisarmReason::CMD_ARM_ZERO;
    //     this->landing_throttle_ = 0.0f;
    //     this->throttle_prev_ = 0.0f;
    //     this->pid_controller_.ResetIntegrator();

    //     this->motor_interface_.SetMotorOutput(0, 0, 0, 0);

    //     this->ChangeState(DroneState::DISARMED);
    //     ESP_LOGW(TAG, "Landing user disarm -> DISARMED");
    //     return ESP_OK;
    // }

    //1. Landing command generation
    AttitudeTarget target{};

    float landing_rate = THROTTLE_RAMP_DOWN_RATE;
    if (this->landing_mode_ == LandingMode::SOFT)
    {
        landing_rate = SOFT_LANDING_RATE;
    }
    else if (this->landing_mode_ == LandingMode::FAILSAFE)
    {
        landing_rate = FAILSAFE_LANDING_RATE;
    }

    const float max_down = landing_rate * dt;
    this->landing_throttle_ = std::max(0.0f, this->landing_throttle_ - max_down);
    target.throttle = this->landing_throttle_;

    if (this->drone_mode_ == DroneMode::ANGLE_SELF_LEVEL)
    {
        target.roll_rad = 0.0f;
        target.pitch_rad = 0.0f;
        target.yaw_rate_rad_s = 0.0f;
        target.roll_rate_rad_s = 0.0f;
        target.pitch_rate_rad_s = 0.0f;
    }
    else
    {
        target.roll_rad = 0.0f;
        target.pitch_rad = 0.0f;
        target.roll_rate_rad_s = 0.0f;
        target.pitch_rate_rad_s = 0.0f;
        target.yaw_rate_rad_s = 0.0f;
    }

    if (target.throttle > 0.0f)
    {
        target.throttle = std::max(target.throttle, IDLE_THROTTLE);
    }

    target.yaw_rad = this->landing_yaw_hold_rad_;
    target.yaw_hold_enable = true;
    target.yaw_rate_rad_s = 0.0f;

    //2. Get IMU data
    IMU_PARESED_DATA imu_data{};
    esp_err_t ret = this->imu_interface_.GetParsedIMURadsData(imu_data);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (this->first_imu_)
    {
        this->gx_lpf_ = imu_data.gx_rad_s;
        this->gy_lpf_ = imu_data.gy_rad_s;
        this->gz_lpf_ = imu_data.gz_rad_s;
        this->ax_lpf_ = imu_data.ax_g;
        this->ay_lpf_ = imu_data.ay_g;
        this->az_lpf_ = imu_data.az_g;
        this->first_imu_ = false;
    }
    else
    {
        this->gx_lpf_ += LPF_GYRO_ALPHA * (imu_data.gx_rad_s - this->gx_lpf_);
        this->gy_lpf_ += LPF_GYRO_ALPHA * (imu_data.gy_rad_s - this->gy_lpf_);
        this->gz_lpf_ += LPF_GYRO_ALPHA * (imu_data.gz_rad_s - this->gz_lpf_);
        this->ax_lpf_ += LPF_ACC_ALPHA * (imu_data.ax_g - this->ax_lpf_);
        this->ay_lpf_ += LPF_ACC_ALPHA * (imu_data.ay_g - this->ay_lpf_);
        this->az_lpf_ += LPF_ACC_ALPHA * (imu_data.az_g - this->az_lpf_);
    }

    imu_data.gx_rad_s = std::clamp(std::isfinite(this->gx_lpf_) ? this->gx_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu_data.gy_rad_s = std::clamp(std::isfinite(this->gy_lpf_) ? this->gy_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu_data.gz_rad_s = std::clamp(std::isfinite(this->gz_lpf_) ? this->gz_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu_data.ax_g = this->ax_lpf_;
    imu_data.ay_g = this->ay_lpf_;
    imu_data.az_g = this->az_lpf_;

    //3. Update EKF and get attitude state
    EKFAttitudeInput ekf_input{};
    ekf_input.ax = imu_data.ax_g;
    ekf_input.ay = imu_data.ay_g;
    ekf_input.az = imu_data.az_g;
    ekf_input.gx = imu_data.gx_rad_s;
    ekf_input.gy = imu_data.gy_rad_s;
    ekf_input.gz = imu_data.gz_rad_s;
    ekf_input.dt = dt;

    const float acc_norm = sqrtf(
        imu_data.ax_g * imu_data.ax_g +
        imu_data.ay_g * imu_data.ay_g +
        imu_data.az_g * imu_data.az_g);

    this->ekf_.Predict(ekf_input);
    if (acc_norm >= IMU_WRONG_DATA_MIN && acc_norm <= IMU_WRONG_DATA_MAX)
    {
        this->ekf_.Update(ekf_input);
    }

    const EKFAttitudeOutput ekf_out = this->ekf_.GetOutput();

    AttitudeState state{};
    state.roll_rad = ekf_out.roll_rad 
                    - this->roll_offset_rad_ 
                    - this->roll_trim_rad_
                    - this->roll_manual_trim_rad_;

    state.pitch_rad = ekf_out.pitch_rad 
                    - this->pitch_offset_rad_ 
                    - this->pitch_trim_rad_
                    - this->pitch_manual_trim_rad_;

    state.yaw_rad = ekf_out.yaw_rad;

    state.gyro_x_rad_s = imu_data.gx_rad_s;
    state.gyro_y_rad_s = imu_data.gy_rad_s;
    state.gyro_z_rad_s = imu_data.gz_rad_s;

    //4. Check tilt safety and update tilt trigger timer
    const bool tilt_exceeded =
        (fabs(state.roll_rad) > TILT_LIMIT_RAD) ||
        (fabs(state.pitch_rad) > TILT_LIMIT_RAD);

    if (tilt_exceeded)
    {
        this->tilt_trigger_dt_ += dt;
    }
    else
    {
        this->tilt_trigger_dt_ = 0.0f;
    }

    //5. PID control
    const bool allow_integrator =
        ((this->state_ == DroneState::LANDING) && this->is_airborne_);

    if (!allow_integrator)
    {
        this->pid_controller_.ResetIntegrator();
    }

    FlightPIDOutput pid_out = this->pid_controller_.Update(target, state, dt, this->output_saturated_);

    //6. PID output limiting
    pid_out.throttle = std::clamp(pid_out.throttle, 0.0f, 1.0f);
    pid_out.roll = std::clamp(pid_out.roll, -0.5f, 0.5f);
    pid_out.pitch = std::clamp(pid_out.pitch, -0.5f, 0.5f);
    pid_out.yaw = std::clamp(pid_out.yaw, -0.5f, 0.5f);

    //7. PID authority reduction at low throttle for better stick centering feel and to prevent ground effect oscillations
    const float roll_pitch_comp = PID_MIN_AUTHORITY_RP + (1.0f - PID_MIN_AUTHORITY_RP) * target.throttle;
    const float yaw_comp = 0.85f + 0.15f * target.throttle;
    // const float yaw_comp = PID_MIN_AUTHORITY_YAW + (1.0f - PID_MIN_AUTHORITY_YAW) * target.throttle;

    pid_out.roll *= roll_pitch_comp;
    pid_out.pitch *= roll_pitch_comp;
    pid_out.yaw *= yaw_comp;

    if (target.throttle > TPA_BREAKPOINT)
    {
        float tpa_ratio = (target.throttle - TPA_BREAKPOINT) / (1.0f - TPA_BREAKPOINT);
        tpa_ratio = std::clamp(tpa_ratio, 0.0f, 1.0f);

        float rp_tpa = 1.0f - TPA_RATE_RP * tpa_ratio;
        float yaw_tpa = 1.0f - TPA_RATE_YAW * tpa_ratio;

        rp_tpa = std::clamp(rp_tpa, 0.5f, 1.0f);
        yaw_tpa = std::clamp(yaw_tpa, 0.5f, 1.0f);

        pid_out.roll *= rp_tpa;
        pid_out.pitch *= rp_tpa;
        pid_out.yaw *= yaw_tpa;
    }

    pid_out.roll = std::clamp(pid_out.roll, -0.5f, 0.5f);
    pid_out.pitch = std::clamp(pid_out.pitch, -0.5f, 0.5f);
    pid_out.yaw = std::clamp(pid_out.yaw, -0.5f, 0.5f);

    // pid_out.roll = 0.0f;
    // pid_out.pitch = 0.0f;
    // pid_out.yaw = 0.0f;

    //8. Thrust curve 
    float t = std::clamp(pid_out.throttle, 0.0f, 1.0f);
    float expo = std::clamp(THRUST_EXPO, 0.0f, 1.0f);

    float thrust_cmd = t * t * (1.0f - expo) + t * expo;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    if (this->state_ == DroneState::ARMED)
    {
        thrust_cmd = std::max(thrust_cmd, IDLE_THROTTLE);
    }

    //9. Voltage compensation
    float voltage = this->battery_monitor_.GetVoltage();

    float v_scale = 1.0f;

    if (voltage > BATTERY_VOLTAGE_MIN_VALID)
    {
        v_scale = BATTERY_VOLTAGE_REF / voltage;
        v_scale = std::clamp(v_scale, 1.0f, V_SCALE_MAX);
    }

    float thrust_v_scale =
        1.0f + (v_scale - 1.0f) * THRUST_VOLT_COMP_GAIN;

    float axis_rp_v_scale =
        1.0f + (v_scale - 1.0f) * AXIS_VOLT_COMP_GAIN_RP;

    float axis_yaw_v_scale =
        1.0f + (v_scale - 1.0f) * AXIS_VOLT_COMP_GAIN_YAW;

    float volt_comp_blend = std::clamp(
        (t - VOLT_COMP_START_THROTTLE) /
        (VOLT_COMP_FULL_THROTTLE - VOLT_COMP_START_THROTTLE),
        0.0f,
        1.0f
    );

    volt_comp_blend = SmoothStep01(volt_comp_blend);

    if (this->is_airborne_)
    {
        volt_comp_blend = 1.0f;
    }

    float final_thrust_v_scale =
        1.0f + (thrust_v_scale - 1.0f) * volt_comp_blend;

    float final_axis_rp_scale =
        1.0f + (axis_rp_v_scale - 1.0f) * volt_comp_blend;

    float final_axis_yaw_scale =
        1.0f + (axis_yaw_v_scale - 1.0f) * volt_comp_blend;

    thrust_cmd *= final_thrust_v_scale;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    //10. Vertical velocity damping (not used at landing)
    float vz_damping =
        -this->vertical_velocity_est_ * K_VZ;

    thrust_cmd += vz_damping;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    // 10.1 High throttle output limiting / sag protection
    float high_thr_blend = std::clamp(
        (t - HIGH_THR_LIMIT_START) /
        (HIGH_THR_LIMIT_FULL - HIGH_THR_LIMIT_START),
        0.0f,
        1.0f
    );

    high_thr_blend = SmoothStep01(high_thr_blend);

    float high_thr_scale =
        1.0f - HIGH_THR_MAX_CUT * high_thr_blend;

    thrust_cmd *= high_thr_scale;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    //11. Mixer bias
    MixerInput mix_in{};
    mix_in.throttle = thrust_cmd;
    // mix_in.roll = pid_out.roll;
    // mix_in.pitch = pid_out.pitch;
    // mix_in.yaw = pid_out.yaw;
    mix_in.roll  = pid_out.roll * final_axis_rp_scale;
    mix_in.pitch = pid_out.pitch * final_axis_rp_scale;
    mix_in.yaw   = pid_out.yaw * final_axis_yaw_scale;

    // VZ based attitude compensation
    float vz_comp = std::clamp(this->vertical_velocity_est_, -1.0f, 1.0f);

    mix_in.roll  += K_ROLL_VZ_COMP  * vz_comp;
    mix_in.pitch += K_PITCH_VZ_COMP * vz_comp;
    mix_in.yaw   += K_YAW_VZ_COMP   * vz_comp;

    // float fl_mul = 0.0f, fr_mul = 0.0f, rr_mul = 0.0f, rl_mul = 0.0f;
    // float mixer_throttle = std::clamp(mix_in.throttle, 0.0f, 1.0f);

    // // hover 기준 
    // constexpr float throttle_low  = 0.3f;
    // constexpr float throttle_high = 0.7f;
    // constexpr float hover_throttle = 0.52f;

    // float throttle_blend = std::clamp(
    //     (mixer_throttle - throttle_low) / (throttle_high - throttle_low),
    //     0.0f,
    //     1.0f);
  
    // float front_bias = std::lerp(
    //     FRONT_THRUST_BIAS * 0.97f,   // LOW
    //     FRONT_THRUST_BIAS * 1.03f,   // HIGH
    //     throttle_blend);

    // float rear_bias = std::lerp(
    //     REAR_THRUST_BIAS * 1.03f,    // LOW
    //     REAR_THRUST_BIAS * 0.97f,    // HIGH
    //     throttle_blend);

    float fl_mul = 0.0f, fr_mul = 0.0f, rr_mul = 0.0f, rl_mul = 0.0f;
    float mixer_throttle = std::clamp(mix_in.throttle, 0.0f, 1.0f);

    // hover 기준 
    constexpr float hover_throttle = 0.52f;

    // hover 기준 차이
    float h_dt = mixer_throttle - hover_throttle;

    // 핵심: 부드러운 비선형
    // float k_gain = h_dt * (1.0f + 3.0f * std::abs(h_dt)); // *2.0f;   // 2.0은 gain, 나중에 튜닝
    float k_gain = std::clamp(h_dt, -0.3f, 0.3f);

    // 보정 강도 
    float h_gain = 0.00f; //0.0654;

    // front / rear 반대 방향으로 보정
    //front_bias = 0.914, rear_bias = 1.086, rises backward
    //front_bias = 0.92, rear_bias = 1.08, rises afterward
    //throttle = 0, -> front_bias = 0.95 +(0.08*-0.52) = 0.9084, 
    //                 rear_bias = 1.05 +(0.08*0.52) = 1.0904
    float front_bias = FRONT_THRUST_BIAS + h_gain * k_gain; 
    float rear_bias  = REAR_THRUST_BIAS  - h_gain * k_gain;

    float left_bias  = LEFT_THRUST_BIAS;
    float right_bias = RIGHT_THRUST_BIAS;
    
    fl_mul = front_bias * left_bias;  // FL
    fr_mul = front_bias * right_bias; // FR
    rr_mul = rear_bias  * right_bias; // RR
    rl_mul = rear_bias  * left_bias;  // RL

    // float throttle_speed = fabsf((thrust_cmd - this->prev_thrust_cmd_) / dt);
    // this->prev_thrust_cmd_ = thrust_cmd;

    // float yaw_ff = -throttle_speed * YAW_THRUST_FF;
    // yaw_ff = std::clamp(yaw_ff, -0.02f, 0.02f);

    // mix_in.yaw += yaw_ff;

    MixerOutput mix_out = MixMotorsWithMultiplier(mix_in, fl_mul, fr_mul, rr_mul, rl_mul);

    MixNormalize(mix_out, 0.0f, 1.0f);
    
    mix_out.m1 = std::clamp(mix_out.m1, 0.0f, 1.0f);
    mix_out.m2 = std::clamp(mix_out.m2, 0.0f, 1.0f);
    mix_out.m3 = std::clamp(mix_out.m3, 0.0f, 1.0f);
    mix_out.m4 = std::clamp(mix_out.m4, 0.0f, 1.0f);

    //12. Ground blend and Normalization
    float ground_throttle = std::clamp(
        (target.throttle - IDLE_THROTTLE) / GROUND_BLEND_RANGE,
        0.0f,
        1.0f);

    float ground_blend = SmoothStep01(ground_throttle);

    ground_blend = 0.15f + 0.85f * ground_blend;

    mix_out.m1 = thrust_cmd + (mix_out.m1 - thrust_cmd) * ground_blend;
    mix_out.m2 = thrust_cmd + (mix_out.m2 - thrust_cmd) * ground_blend;
    mix_out.m3 = thrust_cmd + (mix_out.m3 - thrust_cmd) * ground_blend;
    mix_out.m4 = thrust_cmd + (mix_out.m4 - thrust_cmd) * ground_blend;

    //throttle based gain scheduling for better ground handling
    // float base_throttle = std::clamp(
    // (target.throttle - IDLE_THROTTLE) / (0.70f - IDLE_THROTTLE),
    // 0.0f,
    // 1.0f);

    // base_throttle = base_throttle * base_throttle * (3.0f - 2.0f * base_throttle);

    // float pitch_comp_v = std::lerp(PITCH_COMP_LOW, PITCH_COMP_HIGH, base_throttle);
    // float roll_comp_v  = std::lerp(ROLL_COMP_LOW,  ROLL_COMP_HIGH,  base_throttle);
    // float yaw_comp_v   = std::lerp(YAW_COMP_LOW,   YAW_COMP_HIGH,   base_throttle);

    // float final_p = pitch_comp_v * v_scale;
    // float final_r = roll_comp_v  * v_scale;
    // float final_y = yaw_comp_v   * v_scale;

    // mix_out.m1 += (+final_r - final_p - final_y); // FL
    // mix_out.m2 += (-final_r - final_p + final_y); // FR
    // mix_out.m3 += (-final_r + final_p - final_y); // RR
    // mix_out.m4 += (+final_r + final_p + final_y); // RL


    float avg = 0.25f * (mix_out.m1 + mix_out.m2 + mix_out.m3 + mix_out.m4);
    float delta = thrust_cmd - avg;

    mix_out.m1 += delta;
    mix_out.m2 += delta;
    mix_out.m3 += delta;
    mix_out.m4 += delta;

    MixNormalize(mix_out, 0.0f, 1.0f);

    mix_out.m1 = std::clamp(mix_out.m1, 0.0f, 1.0f);
    mix_out.m2 = std::clamp(mix_out.m2, 0.0f, 1.0f);
    mix_out.m3 = std::clamp(mix_out.m3, 0.0f, 1.0f);
    mix_out.m4 = std::clamp(mix_out.m4, 0.0f, 1.0f);

    //21. DSHOT conversion and motor output
    uint16_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;

    if (target.throttle <= 0.0f)
    {
        d1 = d2 = d3 = d4 = 0;
    }
    else
    {
        d1 = std::max<uint16_t>(ThrottleToDshotWithIdle(mix_out.m1), IDLE_DSHOT_THRESHOLD);
        d2 = std::max<uint16_t>(ThrottleToDshotWithIdle(mix_out.m2), IDLE_DSHOT_THRESHOLD);
        d3 = std::max<uint16_t>(ThrottleToDshotWithIdle(mix_out.m3), IDLE_DSHOT_THRESHOLD);
        d4 = std::max<uint16_t>(ThrottleToDshotWithIdle(mix_out.m4), IDLE_DSHOT_THRESHOLD);
    }

    ret = this->motor_interface_.SetMotorOutput(d1, d2, d3, d4);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "RMT - Motor Control failed.");
        return ret;
    }

    //13. landing completion check
    this->landing_dt_ += dt;

    const bool landing_throttle_low =
        this->landing_throttle_ <= IDLE_THROTTLE + 0.005f;

    const bool landing_done =
        landing_throttle_low &&
        (this->landing_dt_ >= 1.0f || !this->is_airborne_);

    if (landing_done)
    {
        this->landing_throttle_ = 0.0f;
        this->throttle_prev_ = 0.0f;
        this->pid_controller_.ResetIntegrator();
        this->motor_interface_.SetMotorOutput(0, 0, 0, 0);

        this->ChangeState(DroneState::DISARMED);
        ESP_LOGW(TAG, "Landing complete -> DISARMED");
        return ESP_OK;
    }

    //14. Telemetry and logging
    this->tpkt_.roll_rad = state.roll_rad;
    this->tpkt_.pitch_rad = state.pitch_rad;
    this->tpkt_.gyro_x_rad_s = ekf_input.gx;
    this->tpkt_.gyro_y_rad_s = ekf_input.gy;
    this->tpkt_.gyro_z_rad_s = ekf_input.gz;
    this->tpkt_.throttle = target.throttle;

    this->log_.dt = dt;
    this->log_.roll_rad = state.roll_rad;
    this->log_.pitch_rad = state.pitch_rad;
    this->log_.gyro_x_rad_s = ekf_input.gx;
    this->log_.gyro_y_rad_s = ekf_input.gy;
    this->log_.gyro_z_rad_s = ekf_input.gz;
    this->log_.ax_g = ekf_input.ax;
    this->log_.ay_g = ekf_input.ay;
    this->log_.az_g = ekf_input.az;
    this->log_.throttle_cmd = 0.0f;
    this->log_.throttle_used = target.throttle;
    this->log_.roll_out = pid_out.roll;
    this->log_.pitch_out = pid_out.pitch;
    this->log_.yaw_out = pid_out.yaw;
    this->log_.m1 = mix_out.m1;
    this->log_.m2 = mix_out.m2;
    this->log_.m3 = mix_out.m3;
    this->log_.m4 = mix_out.m4;
    this->log_.mode = static_cast<uint8_t>(this->drone_mode_);
    this->log_.state = static_cast<uint8_t>(this->state_);

    return ESP_OK;
}

esp_err_t KSSDrone::BackgroundJobs(const float dt)
{
    this->UpdateCommandCache();
    this->HandleCommandEvents();

    this->battery_check_dt_ += dt;
    if (this->battery_check_dt_ >= CHECK_BAT_MS)
    {
        while (this->battery_check_dt_ >= CHECK_BAT_MS)
        {
            this->battery_check_dt_ -= CHECK_BAT_MS;
        }

        esp_err_t ret = this->battery_monitor_.Update(dt);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    const int64_t rx_dt_us =
        esp_timer_get_time() - this->esp_now_interface_.GetLastRxTimeUs();

    this->telemetry_send_dt_ += dt;

    if (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_MS && rx_dt_us > 5000) //5ms
    {
        while (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_MS)
        {
            this->telemetry_send_dt_ -= CHECK_SEND_TELEMETRY_MS;
        }

        this->tpkt_.mode = static_cast<uint8_t>(this->drone_mode_);
        this->tpkt_.state = static_cast<uint8_t>(this->state_);

        TelemetryPacket send_pkt_ = this->tpkt_;
        
        esp_err_t ret = this->esp_now_interface_.SendTelemetry(send_pkt_);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    if (this->state_ == DroneState::ARMED || this->state_ == DroneState::LANDING)
    {
        this->log_send_dt_ += dt;
        if (this->log_send_dt_ >= CHECK_SEND_LOG_MS)
        {
            while (this->log_send_dt_ >= CHECK_SEND_LOG_MS)
            {
                this->log_send_dt_ -= CHECK_SEND_LOG_MS;
            }
        }
    }

    this->led_controller_.Update(dt);
    return ESP_OK;
}

esp_err_t KSSDrone::Error(const float dt)
{
    (void)dt;
    this->led_controller_.SetLedErrorMode();
    return ESP_OK;
}

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
        return ret;
    }

    ret = this->pid_controller_.Initialize();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->dt_.Initialize();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->imu_interface_.Initialize(&this->bhandle_);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->esp_now_interface_.Initialize();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->battery_monitor_.Initialize();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->led_controller_.Initialize();
    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    ret = this->StartTask();
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ret;
}

void KSSDrone::ChangeState(const DroneState to_state)
{
    if (this->state_ != to_state)
    {
        this->ExitState(this->state_);
        this->EnterState(to_state);
    }
}

void KSSDrone::EnterState(const DroneState to_state)
{
    switch (to_state)
    {
        case DroneState::INIT:
            this->drone_mode_ = DroneMode::RATE_ACRO;
            this->dt_.Reset();
            this->led_controller_.SetLedInitMode();
            break;

        case DroneState::IMU_BIAS_CALIBRATING:
            this->imu_bias_ready_ = false;
            this->ResetGyroBiasAccumulator();
            this->motor_interface_.Disarm();
            this->pid_controller_.Reset();
            this->first_imu_ = true;
            this->led_controller_.SetLedCalibratingMode();
            break;

        case DroneState::DISARMED:

            //imu bias calibration
            this->ResetGyroBiasAccumulator();   
              
            //auto trim
            this->auto_roll_trim_accum_ = 0.0f;
            this->auto_pitch_trim_accum_ = 0.0f;
            this->auto_roll_trim_dt_ = 0.0f;
            this->auto_pitch_trim_dt_ = 0.0f;

            //check airborne
            this->is_airborne_ = false;
            this->airborne_confirm_dt_ = 0.0f;
            this->airborne_exit_confirm_dt_ = 0.0f;

            //armed flag
            this->armed_ = false;

            //seq
            this->control_seq_ready_ = false;
            this->last_control_seq_ = 0;

            this->command_seq_ready_ = false;
            this->last_command_seq_ = 0;

            //throttle ramp
            this->throttle_prev_ = 0.0f;
            this->tilt_trigger_dt_ = 0.0f;
            this->disarmed_settle_dt_ = 0.0f;
            //landing
            this->landing_throttle_ = 0.0f;
            this->landing_dt_ = 0.0f;
            this->landing_mode_ = LandingMode::NONE;
            //motor
            this->motor_interface_.Disarm();
            //pid
            this->pid_controller_.Reset();
            this->output_saturated_ = false;
            //ekf
            this->ekf_.Reset();
            this->ekf_ready_ = false;
            this->ekf_ready_dt_ = 0.0f;
            //imu
            this->first_imu_ = true;
            //esp-now
            this->active_cmd_valid_ = false;
            //led
            this->led_controller_.SetLedDisarmedMode();

            //yaw hold
            this->yaw_hold_initialized_ = false;
            this->yaw_hold_rad_ = 0.0f;

            break;

        case DroneState::ARMING:
            this->arming_check_dt_ = 0.0f;
            this->led_controller_.SetLedArmingMode();
            break;

        case DroneState::ARMED:
            this->armed_ = true;
            this->pid_controller_.Reset();
            this->motor_interface_.Arm();
            this->led_controller_.SetLedArmedMode();
            break;

        case DroneState::LANDING:
        {
            this->landing_dt_ = 0.0f;
            this->landing_throttle_ = this->throttle_prev_;
            this->throttle_prev_ = this->landing_throttle_;
            EKFAttitudeOutput ekf_out = this->ekf_.GetOutput();
            this->landing_yaw_hold_rad_ = ekf_out.yaw_rad;
            this->pid_controller_.Reset();
            this->tilt_trigger_dt_ = 0.0f;
            ESP_LOGW(TAG, "Enter LANDING mode=%u throttle=%.3f",
                     static_cast<unsigned>(this->landing_mode_),
                     this->landing_throttle_);
            break;
        }

        case DroneState::ERR:
            this->led_controller_.SetLedErrorMode();



            break;

        default:
            break;
    }

    this->state_ = to_state;
}

void KSSDrone::ExitState(const DroneState state)
{
    (void)state;
}

void KSSDrone::UpdateCommandCache()
{
    ControlPacket cmd{};
    const bool valid = this->esp_now_interface_.GetLatestCommand(cmd);

    if (!valid)
    {
        this->active_cmd_valid_ = false;
        return;
    }

    bool accept_cmd = false;

    if (!this->control_seq_ready_)
    {
        // First valid packet
        accept_cmd = true;
    }
    else
    {
        const int16_t diff = SeqDiff16(cmd.seq, this->last_control_seq_);

        if (diff > 0)
        {
            // 정상 증가, uint16_t wrap-around 포함
            accept_cmd = true;
        }
        else if (diff == 0)
        {
            // 같은 패킷/중복 패킷.
            // active_cmd_는 그대로 유지한다.
            accept_cmd = false;
        }
        else
        {
            // diff < 0: 오래된 패킷 또는 TX 재부팅/seq reset
            const int32_t seq_drop =
                static_cast<int32_t>(this->last_control_seq_) -
                static_cast<int32_t>(cmd.seq);

            if (this->state_ == DroneState::DISARMED && seq_drop > 100)
            {
                ESP_LOGW(TAG,
                         "TX seq reset accepted in DISARMED: last=%u now=%u drop=%d",
                         this->last_control_seq_,
                         cmd.seq,
                         static_cast<int>(seq_drop));

                accept_cmd = true;

                // 새 TX 세션으로 보는 것이므로 command event seq도 초기화
                this->command_seq_ready_ = false;
                this->last_command_seq_ = 0;
            }
            else
            {
                ESP_LOGW(TAG,
                         "Old control packet ignored: last=%u now=%u diff=%d",
                         this->last_control_seq_,
                         cmd.seq,
                         static_cast<int>(diff));

                accept_cmd = false;
            }
        }
    }

    if (accept_cmd)
    {
        this->last_control_seq_ = cmd.seq;
        this->control_seq_ready_ = true;

        this->active_cmd_ = cmd;
        this->active_cmd_valid_ = true;
    }

    return;
}


void KSSDrone::HandleCommandEvents()
{
    if (!this->active_cmd_valid_)
    {
        return;
    }

    const ControlPacket cmd = this->active_cmd_;

    if (cmd.cmd_flags == CMD_NONE)
    {
        return;
    }

    if (this->command_seq_ready_ &&
        cmd.cmd_seq == this->last_command_seq_)
    {
        return;
    }

    if (!this->command_seq_ready_)
    {
        this->command_seq_ready_ = true;
    }

    this->last_command_seq_ = cmd.cmd_seq;

    if (cmd.cmd_flags & CMD_EMERGENCY_STOP)
    {
        ESP_LOGE(TAG, "EMERGENCY STOP!");

        this->armed_ = false;
        this->landing_throttle_ = 0.0f;
        this->throttle_prev_ = 0.0f;
        this->pid_controller_.ResetIntegrator();
        this->motor_interface_.SetMotorOutput(0, 0, 0, 0);

        this->ChangeState(DroneState::DISARMED);

        return;
    }

    if (cmd.cmd_flags & CMD_LEVEL_CALIBRATE)
    {
        EKFAttitudeOutput ekf_out = this->ekf_.GetOutput();

        const bool stable_enough =
            (std::fabs(ekf_out.bgx) < 0.01f) &&
            (std::fabs(ekf_out.bgy) < 0.01f) &&
            (std::fabs(ekf_out.bgz) < 0.01f);

        const bool angle_reasonable =
            (std::fabs(ekf_out.roll_rad) < 0.7f) &&
            (std::fabs(ekf_out.pitch_rad) < 0.7f);

        const bool safe_to_calibrate =
            (this->imu_bias_ready_ && this->ekf_ready_ && this->state_ == DroneState::DISARMED) &&
            (this->armed_== false) &&
            (cmd.throttle < 0.01f) &&
            stable_enough &&
            angle_reasonable;

        if (safe_to_calibrate)
        {
            this->roll_offset_rad_ = ekf_out.roll_rad;
            this->pitch_offset_rad_ = ekf_out.pitch_rad;

            ESP_LOGI(TAG, "LEVEL CALIBRATED: roll_offset=%.4f pitch_offset=%.4f",
                     this->roll_offset_rad_, this->pitch_offset_rad_);
        }
        else
        {
            ESP_LOGW(TAG, "LEVEL CALIBRATE REJECTED");
        }
        return;
    }

    if (cmd.cmd_flags & CMD_SOFT_LANDING)
    {
        if (this->state_ == DroneState::ARMED)
        {
            this->debug_disarm_reason_ = DisarmReason::LANDING_EMERGENCY;
            this->landing_mode_ = LandingMode::SOFT;
            this->ChangeState(DroneState::LANDING);
            ESP_LOGW(TAG, "Soft landing requested ->LANDING(SOFT)");
        }
        else
        {
            ESP_LOGW(TAG, "Soft landing rejected: not in ARMED");
        }

        return;
    }

    if (cmd.cmd_flags & CMD_GYRO_CALIBRATE)
    {
        ESP_LOGI(TAG, "GYRO CALIBRATE requested");
    }

    if (cmd.cmd_flags & CMD_SET_MODE)
    {
        if (cmd.mode > static_cast<uint8_t>(DroneMode::ANGLE_SELF_LEVEL))
        {
            ESP_LOGW(TAG, "Invalid drone mode: %u", cmd.mode);
        }
        else if (this->state_ == DroneState::DISARMED)
        {
            const DroneMode old_mode = this->drone_mode_;
            const DroneMode new_mode = static_cast<DroneMode>(cmd.mode);

            if (old_mode != new_mode)
            {
                this->pid_controller_.Reset();
                this->throttle_prev_ = 0.0f;
                this->output_saturated_ = false;
                this->drone_mode_ = new_mode;

                ESP_LOGI(TAG, "Drone Mode Changed: %u -> %u",
                        static_cast<unsigned>(old_mode),
                        static_cast<unsigned>(new_mode));
            }
        }
        else
        {
            ESP_LOGW(TAG, "Mode change rejected: only allowed in DISARMED");
        }
    }

    if (cmd.cmd_flags & CMD_ARM_REQUEST)
    {
        const bool safe_to_arm =
            this->state_ == DroneState::DISARMED &&
            this->imu_bias_ready_ &&
            this->ekf_ready_ &&
            this->disarmed_settle_dt_ >= DISARM_SETTLE_TIME &&
            cmd.throttle < 0.05f &&
            this->armed_ == false;

        if (safe_to_arm)
        {
            this->armed_ = true;
            this->pid_controller_.Reset();
            this->throttle_prev_ = 0.0f;
            this->output_saturated_ = false;
        }
        else
        {
            ESP_LOGW(TAG, "ARM_REQUEST rejected");
        }        
        return;
    }

    if (cmd.cmd_flags & CMD_DISARM_REQUEST)
    {
        this->armed_ = false;
        
        return;
    }

    return;
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

void KSSDrone::PktDataStore() {}
void KSSDrone::LogDataStore() {}

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

void KSSDrone::UpdateDynamicClimbTrim(
    const float dt,
    const AttitudeTarget& target,
    const float throttle_rate_cmd)
{
    (void)dt;

    float climb_blend = 0.0f;

    if (this->is_airborne_ &&
        target.throttle > CLIMB_TRIM_MIN_THROTTLE &&
        throttle_rate_cmd > CLIMB_TRIM_THR_RATE_START)
    {
        climb_blend = std::clamp(
            (throttle_rate_cmd - CLIMB_TRIM_THR_RATE_START) /
            (CLIMB_TRIM_THR_RATE_FULL - CLIMB_TRIM_THR_RATE_START),
            0.0f,
            1.0f);

        climb_blend = SmoothStep01(climb_blend);
    }

    const float roll_trim_target =
        CLIMB_ROLL_TRIM_MAX * climb_blend;

    const float pitch_trim_target =
        CLIMB_PITCH_TRIM_MAX * climb_blend;

    this->dynamic_roll_trim_rad_ +=
        CLIMB_TRIM_LPF_ALPHA *
        (roll_trim_target - this->dynamic_roll_trim_rad_);

    this->dynamic_pitch_trim_rad_ +=
        CLIMB_TRIM_LPF_ALPHA *
        (pitch_trim_target - this->dynamic_pitch_trim_rad_);
}

void KSSDrone::UpdateDynamicTakeoffTrim(const AttitudeTarget& target, const float throttle_rate_cmd)
{
    float roll_trim_target = 0.0f;
    float pitch_trim_target = 0.0f;

    if (!this->is_airborne_ &&
        target.throttle > TAKEOFF_TRIM_THR_MIN &&
        throttle_rate_cmd > TAKEOFF_TRIM_THR_RATE_START)
    {
        float blend = std::clamp(
            (throttle_rate_cmd - TAKEOFF_TRIM_THR_RATE_START) /
            (TAKEOFF_TRIM_THR_RATE_FULL - TAKEOFF_TRIM_THR_RATE_START),
            0.0f,
            1.0f);

        blend = SmoothStep01(blend);

        roll_trim_target  = TAKEOFF_ROLL_TRIM_MAX * blend;
        pitch_trim_target = TAKEOFF_PITCH_TRIM_MAX * blend;
    }

    const bool roll_trim_increasing =
        std::fabs(roll_trim_target) > std::fabs(this->takeoff_roll_trim_rad_);

    const float alpha = roll_trim_increasing ?
        TAKEOFF_TRIM_ATTACK_ALPHA :
        TAKEOFF_TRIM_RELEASE_ALPHA;

    this->takeoff_roll_trim_rad_ +=
        alpha * (roll_trim_target - this->takeoff_roll_trim_rad_);

    this->takeoff_pitch_trim_rad_ +=
        alpha * (pitch_trim_target - this->takeoff_pitch_trim_rad_);
}