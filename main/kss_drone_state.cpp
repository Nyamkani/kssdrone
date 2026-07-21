#include "kss_drone.h"

static const char* TAG = "kss_drone";

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

/*
 * 기존 Armed() 재구성
 */

esp_err_t KSSDrone::Armed(const float dt)
{
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;

    ArmedContext ctx{};
    this->LoadArmedContextCache(ctx);

    float failsafe_dt = 0.0f;
    if (ConsumePeriodicDt(this->armed_failsafe_dt_accum_,
                        dt,
                        ARMED_FAILSAFE_PERIOD_SEC,
                        failsafe_dt))
    {
        ret = this->ArmedFailsafeFast(failsafe_dt);
        if (ret != ESP_OK || this->state_ != DroneState::ARMED)
        {
            return ret;
        }
    }

    ret = this->ArmedCommandMedium(dt, ctx);
    if (ret != ESP_OK || this->state_ != DroneState::ARMED)
    {
        return ret;
    }

    ret = this->ArmedImuFast(dt, ctx);
    if (ret != ESP_OK || this->state_ != DroneState::ARMED)
    {
        return ret;
    }

    ret = this->ArmedEstimatorMedium(dt, ctx);
    if (ret != ESP_OK || this->state_ != DroneState::ARMED)
    {
        return ret;
    }

    this->ArmedAirborneSlow(dt, ctx);
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    this->ArmedSlowCompUpdate(dt, ctx);
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    float tilt_dt = 0.0f;
    if (ConsumePeriodicDt(this->armed_tilt_safety_dt_accum_,
                        dt,
                        ARMED_TILT_SAFETY_PERIOD_SEC,
                        tilt_dt))
    {
        this->ArmedTiltSafetyMedium(tilt_dt, ctx);
        if (this->state_ != DroneState::ARMED)
        {
            return ESP_OK;
        }
    }

    /*
     * 2kHz PID
     */
    float control_dt = 0.0f;
    if (ConsumePeriodicDt(this->armed_control_dt_accum_,
                          dt,
                          ARMED_CONTROL_PERIOD_SEC,
                          control_dt))
    {
        ret = this->ArmedControlFast(control_dt, ctx);
        if (ret != ESP_OK || this->state_ != DroneState::ARMED)
        {
            return ret;
        }
    }

    /*
     * output compensation은 PID 이후에 실행.
     * ctx.pid_out 또는 armed_pid_out_cache_를 기준으로 계산할 수 있음.
     */
    this->ArmedOutputCompMedium(dt, ctx);
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    /*
     * 1kHz DShot output
     */
    float motor_dt = 0.0f;
    if (ConsumePeriodicDt(this->armed_motor_output_dt_accum_,
                          dt,
                          ARMED_MOTOR_OUTPUT_PERIOD_SEC,
                          motor_dt))
    {
        if (this->armed_pid_valid_)
        {
            ctx.pid_out = this->armed_pid_out_cache_;

            ret = this->ArmedOutputFast(motor_dt, ctx);
            if (ret != ESP_OK)
            {
                return ret;
            }
        }
    }

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

esp_err_t KSSDrone::Error(const float dt)
{
    (void)dt;
    this->led_controller_.SetLedErrorMode();
    return ESP_OK;
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

            this->ResetArmedRuntime();

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
            this->ResetArmedRuntime();

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