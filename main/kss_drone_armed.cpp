#include "kss_drone.h"

static const char* TAG = "kss_drone";

esp_err_t KSSDrone::ArmedPrepareFast(const float dt, ArmedContext& ctx)
{
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

    ctx.cmd = &this->active_cmd_;
    const ControlPacket& cmd = *ctx.cmd;

    // if (this->battery_monitor_.GetState() == BatteryState::CRITICAL)
    // {
    //     this->debug_disarm_reason_ = DisarmReason::BATTERY_CRITICAL;
    //     this->landing_mode_ = LandingMode::FAILSAFE;
    //     this->ChangeState(DroneState::LANDING);
    //     ESP_LOGE(TAG, "CRITICAL battery! -> LANDING(FAILSAFE)");
    //     return ESP_OK;
    // }

    if (!this->armed_)
    {
        const bool likely_airborne =
            this->is_airborne_ ||
            this->throttle_prev_ > (IDLE_THROTTLE + 0.05f);

        if (likely_airborne)
        {
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

    this->BuildAttitudeTarget(cmd, ctx.target);

    ctx.target.roll_rad =
        SlewLimit(this->roll_target_smooth_rad_,
                  ctx.target.roll_rad,
                  TARGET_ANGLE_SLEW_RATE,
                  dt);

    ctx.target.pitch_rad =
        SlewLimit(this->pitch_target_smooth_rad_,
                  ctx.target.pitch_rad,
                  TARGET_ANGLE_SLEW_RATE,
                  dt);

    ctx.target.yaw_rate_rad_s =
        SlewLimit(this->yaw_rate_target_smooth_rad_s_,
                  ctx.target.yaw_rate_rad_s,
                  TARGET_YAW_SLEW_RATE,
                  dt);

    this->roll_target_smooth_rad_ = ctx.target.roll_rad;
    this->pitch_target_smooth_rad_ = ctx.target.pitch_rad;
    this->yaw_rate_target_smooth_rad_s_ = ctx.target.yaw_rate_rad_s;

    const bool throttle_cut =
        ctx.target.throttle <= CUT_OFF_THROTTLE;

    if (throttle_cut)
    {
        ctx.target.throttle = 0.0f;
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

        float delta = ctx.target.throttle - this->throttle_prev_;

        if (delta > 0.0f)
        {
            delta = std::min(delta, max_up);
        }
        else
        {
            delta = std::max(delta, -max_down);
        }

        ctx.target.throttle = this->throttle_prev_ + delta;
        this->throttle_prev_ = ctx.target.throttle;

        ctx.target.throttle =
            std::max(ctx.target.throttle, IDLE_THROTTLE);

        ctx.throttle_rate_cmd = delta / dt;
    }

    SharedSnapshotFrame<IMU_PARESED_DATA> imu_frame{};

    esp_err_t ret = this->imu_interface_.GetParsedIMURadsDataWithFrame(imu_frame);
    if (ret != ESP_OK)
    {
        return ret;
    }

    const int64_t now_us = esp_timer_get_time();

#if ENABLE_DRONE_MAIN_STATS_LOG
    this->UpdateImuFrameStats(imu_frame, now_us);
#endif
    ctx.imu = imu_frame.data;

    if (this->first_imu_)
    {
        this->gx_lpf_ = ctx.imu.gx_rad_s;
        this->gy_lpf_ = ctx.imu.gy_rad_s;
        this->gz_lpf_ = ctx.imu.gz_rad_s;

        this->ax_lpf_ = ctx.imu.ax_g;
        this->ay_lpf_ = ctx.imu.ay_g;
        this->az_lpf_ = ctx.imu.az_g;

        this->first_imu_ = false;
    }
    else
    {
        this->gx_lpf_ += LPF_GYRO_ALPHA * (ctx.imu.gx_rad_s - this->gx_lpf_);
        this->gy_lpf_ += LPF_GYRO_ALPHA * (ctx.imu.gy_rad_s - this->gy_lpf_);
        this->gz_lpf_ += LPF_GYRO_ALPHA * (ctx.imu.gz_rad_s - this->gz_lpf_);

        this->ax_lpf_ += LPF_ACC_ALPHA * (ctx.imu.ax_g - this->ax_lpf_);
        this->ay_lpf_ += LPF_ACC_ALPHA * (ctx.imu.ay_g - this->ay_lpf_);
        this->az_lpf_ += LPF_ACC_ALPHA * (ctx.imu.az_g - this->az_lpf_);
    }

    ctx.imu.gx_rad_s = std::clamp(std::isfinite(this->gx_lpf_) ? this->gx_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    ctx.imu.gy_rad_s = std::clamp(std::isfinite(this->gy_lpf_) ? this->gy_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    ctx.imu.gz_rad_s = std::clamp(std::isfinite(this->gz_lpf_) ? this->gz_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);

    ctx.imu.ax_g = this->ax_lpf_;
    ctx.imu.ay_g = this->ay_lpf_;
    ctx.imu.az_g = this->az_lpf_;

    ctx.ekf_input.ax = ctx.imu.ax_g;
    ctx.ekf_input.ay = ctx.imu.ay_g;
    ctx.ekf_input.az = ctx.imu.az_g;
    ctx.ekf_input.gx = ctx.imu.gx_rad_s;
    ctx.ekf_input.gy = ctx.imu.gy_rad_s;
    ctx.ekf_input.gz = ctx.imu.gz_rad_s;
    ctx.ekf_input.dt = dt;

    const float acc_norm = sqrtf(
        ctx.imu.ax_g * ctx.imu.ax_g +
        ctx.imu.ay_g * ctx.imu.ay_g +
        ctx.imu.az_g * ctx.imu.az_g
    );

    this->ekf_.Predict(ctx.ekf_input);

    if (acc_norm >= IMU_WRONG_DATA_MIN && acc_norm <= IMU_WRONG_DATA_MAX)
    {
        this->ekf_.Update(ctx.ekf_input);
    }

    ctx.ekf_out = this->ekf_.GetOutput();

    const bool yaw_stick_active =
        std::fabs(cmd.yaw_rate_rad_s) > YAW_STICK_DEADBAND_RAD_S;

    if (!this->yaw_hold_initialized_)
    {
        this->yaw_hold_rad_ = ctx.ekf_out.yaw_rad;
        this->yaw_hold_initialized_ = true;
    }

    if (yaw_stick_active)
    {
        this->yaw_hold_rad_ = ctx.ekf_out.yaw_rad;
        ctx.target.yaw_hold_enable = false;
    }
    else
    {
        ctx.target.yaw_rate_rad_s = 0.0f;
        ctx.target.yaw_hold_enable = true;
    }

    ctx.target.yaw_rad = this->yaw_hold_rad_;

    this->UpdateDynamicTakeoffTrim(ctx.target, ctx.throttle_rate_cmd);

    ctx.state.roll_rad =
        ctx.ekf_out.roll_rad
        - this->roll_offset_rad_
        - this->roll_trim_rad_
        - this->roll_manual_trim_rad_
        - this->takeoff_roll_trim_rad_
        - this->dynamic_roll_trim_rad_;

    ctx.state.pitch_rad =
        ctx.ekf_out.pitch_rad
        - this->pitch_offset_rad_
        - this->pitch_trim_rad_
        - this->pitch_manual_trim_rad_
        - this->takeoff_pitch_trim_rad_
        - this->dynamic_pitch_trim_rad_;

    ctx.state.yaw_rad = ctx.ekf_out.yaw_rad;

    ctx.state.gyro_x_rad_s = ctx.imu.gx_rad_s;
    ctx.state.gyro_y_rad_s = ctx.imu.gy_rad_s;
    ctx.state.gyro_z_rad_s = ctx.imu.gz_rad_s;

    this->UpdateAirborneState(dt, ctx.target);

    return ESP_OK;
}

void KSSDrone::ArmedSlowCompUpdate(const float dt, ArmedContext& ctx)
{
    this->armed_slow_comp_dt_accum_ += dt;

    if (this->armed_slow_comp_dt_accum_ < ARMED_SLOW_COMP_TIME)
    {
        return;
    }

    const float slow_dt = this->armed_slow_comp_dt_accum_;
    this->armed_slow_comp_dt_accum_ = 0.0f;

    this->UpdatePseudoVelocitySlow(slow_dt, ctx);

    this->UpdateVoltageCompSlow(ctx);

    this->UpdateAutoTrimSlow(slow_dt, ctx);
}

esp_err_t KSSDrone::ArmedControlFast(const float dt, ArmedContext& ctx)
{
    /*
     * Slow compensation result 적용.
     * 계산은 250Hz, 적용은 매 loop.
     */
    ctx.state.pitch_rad -= this->cached_pitch_vel_comp_rad_;
    ctx.state.roll_rad  -= this->cached_roll_vel_comp_rad_;

    const bool tilt_exceeded =
        (fabs(ctx.state.roll_rad) > TILT_LIMIT_RAD) ||
        (fabs(ctx.state.pitch_rad) > TILT_LIMIT_RAD);

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

    const bool allow_integrator =
        ((this->state_ == DroneState::ARMED) && this->is_airborne_);

    if (!allow_integrator)
    {
        this->pid_controller_.ResetIntegrator();
    }

    ctx.pid_out =
        this->pid_controller_.Update(
            ctx.target,
            ctx.state,
            dt,
            this->output_saturated_
        );

    /*
     * 이후는 기존 12~20번:
     * - PID output limiting
     * - low throttle authority
     * - yaw priority / yaw hover boost
     * - thrust curve
     * - cached voltage scale 적용
     * - vertical damping
     * - mixer
     * - DShot conversion
     * - SetMotorOutput
     * - telemetry/log 값 채우기
     */

    return this->ArmedOutputFast(dt, ctx);
}

esp_err_t KSSDrone::ArmedOutputFast(const float dt, ArmedContext& ctx)
{
    //12. PID output limiting
    ctx.pid_out.throttle = std::clamp(ctx.pid_out.throttle, 0.0f, 1.0f);
    ctx.pid_out.roll = std::clamp(ctx.pid_out.roll, -0.5f, 0.5f);
    ctx.pid_out.pitch = std::clamp(ctx.pid_out.pitch, -0.5f, 0.5f);
    ctx.pid_out.yaw = std::clamp(ctx.pid_out.yaw, -0.5f, 0.5f);

    // Low throttle / pre-air authority
    if (!this->is_airborne_)
    {
        const float preair_throttle = std::clamp(
            (ctx.target.throttle - IDLE_THROTTLE) /
            (AIRBORNE_THROTTLE_ENTER - IDLE_THROTTLE),
            0.0f,
            1.0f);

        const float preair_scale = 0.80f + 0.20f * preair_throttle;
        const float yaw_preair_scale = std::lerp(0.60f, 1.0f, preair_throttle);

        ctx.pid_out.roll  *= preair_scale;
        ctx.pid_out.pitch *= preair_scale;
        ctx.pid_out.yaw   *= yaw_preair_scale;

        this->vertical_velocity_est_ = 0.0f;
    }

    //13. PID authority reduction at low throttle for better stick centering feel and to prevent ground effect oscillations
    //7. PID authority reduction at low throttle for better stick centering feel and to prevent ground effect oscillations
    const float roll_pitch_comp = PID_MIN_AUTHORITY_RP + (1.0f - PID_MIN_AUTHORITY_RP) * ctx.target.throttle;
    const float yaw_comp = 0.85f + 0.15f * ctx.target.throttle;
    // const float yaw_comp = PID_MIN_AUTHORITY_YAW + (1.0f - PID_MIN_AUTHORITY_YAW) * target.throttle;

    ctx.pid_out.roll *= roll_pitch_comp;
    ctx.pid_out.pitch *= roll_pitch_comp;
    ctx.pid_out.yaw *= yaw_comp;

    ctx.pid_out.roll = std::clamp(ctx.pid_out.roll, -0.5f, 0.5f);
    ctx.pid_out.pitch = std::clamp(ctx.pid_out.pitch, -0.5f, 0.5f);
    ctx.pid_out.yaw = std::clamp(ctx.pid_out.yaw, -0.5f, 0.5f);


    float rp_activity =
        std::max(std::fabs(ctx.pid_out.roll), std::fabs(ctx.pid_out.pitch));

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
        std::fabs(ctx.throttle_rate_cmd) / YAW_HOVER_THR_RATE_REF,
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
    ctx.pid_out.yaw *= yaw_hover_boost;

    // 2. roll/pitch가 바빠지면 yaw 양보
    ctx.pid_out.yaw *= yaw_priority_scale;

    // 3. 최종 yaw clamp
    ctx.pid_out.yaw = std::clamp(ctx.pid_out.yaw, -0.5f, 0.5f);

    // pid_out.roll = 0.0f;
    // pid_out.pitch = 0.0f;
    // pid_out.yaw = 0.0f;

    //14. Thrust curve 
    float t = std::clamp(ctx.pid_out.throttle, 0.0f, 1.0f);
    float expo = std::clamp(THRUST_EXPO, 0.0f, 1.0f);

    float thrust_cmd = t * t * (1.0f - expo) + t * expo;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    if (this->state_ == DroneState::ARMED)
    {
        thrust_cmd = std::max(thrust_cmd, IDLE_THROTTLE);
    }

    //15. Voltage compensation
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

    float thrust_v_scale = this->cached_thrust_v_scale_;
    float axis_rp_v_scale = this->cached_axis_rp_v_scale_;
    float axis_yaw_v_scale = this->cached_axis_yaw_v_scale_;

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
    mix_in.roll  = ctx.pid_out.roll * final_axis_rp_scale;
    mix_in.pitch = ctx.pid_out.pitch * final_axis_rp_scale;
    mix_in.yaw   = ctx.pid_out.yaw * final_axis_yaw_scale;

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

    if (this->is_airborne_ && ctx.throttle_rate_cmd < 0.0f)
    {
        yaw_throttle_down_ff =
            -ctx.throttle_rate_cmd * YAW_THR_DOWN_FF_GAIN;

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
            (ctx.target.throttle - IDLE_THROTTLE) / GROUND_BLEND_RANGE,
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

    esp_err_t ret = this->motor_interface_.SetMotorOutput(d1, d2, d3, d4);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "RMT - Motor Control failed.");
        return ret;
    }

    //20. Telemetry and logging
    this->tpkt_.roll_rad = ctx.state.roll_rad;
    this->tpkt_.pitch_rad = ctx.state.pitch_rad;

    this->tpkt_.gyro_x_rad_s = ctx.ekf_input.gx;
    this->tpkt_.gyro_y_rad_s = ctx.ekf_input.gy;
    this->tpkt_.gyro_z_rad_s = ctx.ekf_input.gz;

    this->tpkt_.throttle = ctx.target.throttle;

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
    this->log_.roll_rad = ctx.state.roll_rad;
    this->log_.pitch_rad = ctx.state.pitch_rad;
    this->log_.gyro_x_rad_s = ctx.ekf_input.gx;
    this->log_.gyro_y_rad_s = ctx.ekf_input.gy;
    this->log_.gyro_z_rad_s = ctx.ekf_input.gz;
    this->log_.ax_g = ctx.ekf_input.ax;
    this->log_.ay_g = ctx.ekf_input.ay;
    this->log_.az_g = ctx.ekf_input.az;
    this->log_.throttle_cmd = ctx.cmd->throttle;
    this->log_.throttle_used = ctx.target.throttle;
    this->log_.roll_out = ctx.pid_out.roll;
    this->log_.pitch_out = ctx.pid_out.pitch;
    this->log_.yaw_out = ctx.pid_out.yaw;
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

