#include "kss_drone.h"

static const char* TAG = "kss_drone";

void KSSDrone::LoadArmedContextCache(ArmedContext& ctx)
{
    if (this->armed_cmd_valid_)
    {
        ctx.cmd = &this->active_cmd_;
        ctx.cmd_valid = true;
    }

    if (this->armed_target_valid_)
    {
        ctx.target = this->armed_target_cache_;
        ctx.target_valid = true;
    }

    if (this->armed_imu_valid_)
    {
        ctx.imu = this->armed_imu_cache_;
        ctx.imu_valid = true;
    }

    if (this->armed_ekf_valid_)
    {
        ctx.ekf_input = this->armed_ekf_input_cache_;
        ctx.ekf_out = this->armed_ekf_out_cache_;
        ctx.ekf_valid = true;
    }

    if (this->armed_state_valid_)
    {
        ctx.state = this->armed_state_cache_;
        ctx.state_valid = true;
    }

    ctx.throttle_rate_cmd = this->armed_throttle_rate_cmd_;
}


void KSSDrone::ResetArmedRuntime()
{
    this->armed_cmd_valid_ = false;
    this->armed_target_valid_ = false;
    this->armed_imu_valid_ = false;
    this->armed_ekf_valid_ = false;
    this->armed_state_valid_ = false;


    this->armed_target_cache_ = {};
    this->armed_imu_cache_ = {};
    this->armed_ekf_input_cache_ = {};
    this->armed_ekf_out_cache_ = {};
    this->armed_state_cache_ = {};
    this->armed_output_cache_ = {};


    this->armed_throttle_rate_cmd_ = 0.0f;

    this->armed_command_dt_accum_ = ARMED_COMMAND_PERIOD_SEC;
    this->armed_estimator_dt_accum_ = ARMED_ESTIMATOR_PERIOD_SEC;
    this->armed_failsafe_dt_accum_ = ARMED_FAILSAFE_PERIOD_SEC;
    this->armed_airborne_dt_accum_ = ARMED_AIRBORNE_PERIOD_SEC;
    this->armed_slow_comp_dt_accum_ = ARMED_SOFT_COMP_PERIOD_SEC;
    this->armed_output_comp_dt_accum_ = ARMED_OUTPUT_COMP_PERIOD_SEC;
    this->armed_control_dt_accum_ = ARMED_CONTROL_PERIOD_SEC;
    this->armed_motor_output_dt_accum_ = ARMED_MOTOR_OUTPUT_PERIOD_SEC;

    this->armed_last_cmd_us_ = 0;
    this->armed_log_dt_accum_ = 0.0f;

    this->armed_pid_valid_ = false;
    this->armed_pid_out_cache_ = {};

}

/*
 * 1. Failsafe fast
 *
 * 주의:
 * 여기서는 esp-now snapshot을 매번 읽지 않는다.
 * ArmedCommandMedium()에서 갱신한 armed_last_cmd_us_만 확인한다.
 */

esp_err_t KSSDrone::ArmedFailsafeFast(const float dt)
{
    (void)dt;

    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    if (this->armed_last_cmd_us_ <= 0)
    {
        /*
         * 아직 command medium이 한 번도 성공하지 않은 상태.
         * 첫 command 획득은 ArmedCommandMedium()에서 처리한다.
         */
        return ESP_OK;
    }

     const int64_t now_us = esp_timer_get_time();

     //#define COMMAND_TIMEOUT_US    300000
    if ((now_us - this->armed_last_cmd_us_) > COMMAND_TIMEOUT_US)
    {
        // ESP_LOGE(TAG,
        //     "Command timeout -> LANDING dt_ms=%lld rx=%lu seq=%lu max_gap_ms=%lld jump=%lu",
        //     (esp_timer_get_time() - this->esp_now_interface_.GetLastRxTimeUs()) / 1000,
        //     this->esp_now_interface_.GetRxCount(),
        //     this->esp_now_interface_.GetLastSeq(),
        //     this->esp_now_interface_.GetMaxRxGapUs() / 1000,
        //     this->esp_now_interface_.GetSeqJumpCount()
        // );

        this->debug_disarm_reason_ = DisarmReason::COMM_TIMEOUT;
        this->landing_mode_ = LandingMode::FAILSAFE;
        this->ChangeState(DroneState::LANDING);
        return ESP_OK;
    }

    if (this->battery_monitor_.GetState() == BatteryState::CRITICAL)
    {
        this->debug_disarm_reason_ = DisarmReason::BATTERY_CRITICAL;
        this->landing_mode_ = LandingMode::FAILSAFE;
        this->ChangeState(DroneState::LANDING);
        ESP_LOGE(TAG, "CRITICAL battery! -> LANDING(FAILSAFE)");
        return ESP_OK;
    }

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

    return ESP_OK;
}

/*
 * 2. Command medium, 500Hz
 *
 * 기존 ArmedPrepareFast()에서 아래 부분을 이 함수로 이동:
 * - esp-now latest command read
 * - command packet 변환
 * - disarm / landing command event
 * - slew
 * - throttle ramp
 * - limit
 * - target roll/pitch/yaw/throttle 생성
 */

esp_err_t KSSDrone::ArmedCommandMedium(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    float cmd_dt = 0.0f;
    if (!ConsumePeriodicDt(this->armed_command_dt_accum_,
                           dt,
                           ARMED_COMMAND_PERIOD_SEC,
                           cmd_dt))
    {
        return ESP_OK;
    }

    // if (!this->active_cmd_valid_ || this->esp_now_interface_.IsTimeout())
     if (!this->active_cmd_valid_ || this->crsf_receiver_.IsTimeout())
    {
        this->armed_cmd_valid_ = false;
        this->armed_target_valid_ = false;

        this->debug_disarm_reason_ = DisarmReason::COMM_TIMEOUT;
        this->landing_mode_ = LandingMode::FAILSAFE;
        this->ChangeState(DroneState::LANDING);
        return ESP_OK;
    }


    /*
     * command는 background에서 active_cmd_로 갱신된다면
     * 여기서는 active_cmd_를 사용한다.
     * GetLatestCommand()를 여기서 직접 부르는 구조라면 여기에서 읽는다.
     */
    this->armed_cmd_valid_ = true;
    ctx.cmd = &this->active_cmd_;
    ctx.cmd_valid = true;

    const ControlPacket& cmd = *ctx.cmd;

    // this->armed_last_cmd_us_ =
    //     this->esp_now_interface_.GetLastRxTimeUs();
    this->armed_last_cmd_us_ =
        this->crsf_receiver_.GetLastRcTimeUs();

    if (!this->armed_)
    {
        const bool likely_airborne =
            this->is_airborne_ ||
            this->throttle_prev_ > (IDLE_THROTTLE + 0.05f);

        if (!likely_airborne)
        {
            this->debug_disarm_reason_ = DisarmReason::CMD_ARM_ZERO;
            this->ChangeState(DroneState::DISARMED);
        }

        return ESP_OK;
    }

    AttitudeTarget target{};

    this->BuildAttitudeTarget(cmd, target);

    target.roll_rad =
        SlewLimit(this->roll_target_smooth_rad_,
                  target.roll_rad,
                  TARGET_ANGLE_SLEW_RATE,
                  cmd_dt);

    target.pitch_rad =
        SlewLimit(this->pitch_target_smooth_rad_,
                  target.pitch_rad,
                  TARGET_ANGLE_SLEW_RATE,
                  cmd_dt);

    target.yaw_rate_rad_s =
        SlewLimit(this->yaw_rate_target_smooth_rad_s_,
                  target.yaw_rate_rad_s,
                  TARGET_YAW_SLEW_RATE,
                  cmd_dt);

    this->roll_target_smooth_rad_ = target.roll_rad;
    this->pitch_target_smooth_rad_ = target.pitch_rad;
    this->yaw_rate_target_smooth_rad_s_ = target.yaw_rate_rad_s;

    const bool throttle_cut =
        target.throttle <= CUT_OFF_THROTTLE;

    float throttle_rate_cmd = 0.0f;

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

        const float max_up = ramp_up_rate * cmd_dt;
        const float max_down = THROTTLE_RAMP_DOWN_RATE * cmd_dt;

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

        target.throttle =
            std::max(target.throttle, IDLE_THROTTLE);

        throttle_rate_cmd = delta / cmd_dt;
    }

    ctx.target = target;
    ctx.target_valid = true;
    ctx.throttle_rate_cmd = throttle_rate_cmd;
    ctx.command_updated = true;

    this->armed_target_cache_ = target;
    this->armed_target_valid_ = true;
    this->armed_throttle_rate_cmd_ = throttle_rate_cmd;

    return ESP_OK;
}
/*
 * 3. IMU fast
 *
 * 기존 ArmedPrepareFast()에서 아래 부분을 이 함수로 이동:
 * - IMU direct read
 * - raw convert
 * - gyro/acc 1차 LPF
 *
 * EKF는 여기서 하지 않는다.
 */

esp_err_t KSSDrone::ArmedImuFast(const float dt, ArmedContext& ctx)
{
    (void)ctx;

    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    /*
     * 기존 IMU 수신 + 1차 LPF 코드를 여기에 이동.
     *
     * 예:
     * ret = this->imu_interface_.GetControlImuFrame(...);
     * this->ApplyGyroLpf(...);
     * this->ApplyAccelLpf(...);
     */
    SharedSnapshotFrame<IMU_PARESED_DATA> imu_frame{};

    esp_err_t ret = this->imu_interface_.GetParsedIMURadsDataWithFrame(imu_frame);
    if (ret != ESP_OK)
    {
        ctx.imu_valid = false;

        return ret;
    }

#if ENABLE_DRONE_MAIN_STATS_LOG
    const int64_t now_us = esp_timer_get_time();
    this->UpdateImuFrameStats(imu_frame, now_us);
#endif

    IMU_PARESED_DATA imu = imu_frame.data;

    if (this->first_imu_)
    {
        this->gx_lpf_ = imu.gx_rad_s;
        this->gy_lpf_ = imu.gy_rad_s;
        this->gz_lpf_ = imu.gz_rad_s;

        this->ax_lpf_ = imu.ax_g;
        this->ay_lpf_ = imu.ay_g;
        this->az_lpf_ = imu.az_g;

        this->first_imu_ = false;
    }
    else
    {
        this->gx_lpf_ += LPF_GYRO_ALPHA * (imu.gx_rad_s - this->gx_lpf_);
        this->gy_lpf_ += LPF_GYRO_ALPHA * (imu.gy_rad_s - this->gy_lpf_);
        this->gz_lpf_ += LPF_GYRO_ALPHA * (imu.gz_rad_s - this->gz_lpf_);

        this->ax_lpf_ += LPF_ACC_ALPHA * (imu.ax_g - this->ax_lpf_);
        this->ay_lpf_ += LPF_ACC_ALPHA * (imu.ay_g - this->ay_lpf_);
        this->az_lpf_ += LPF_ACC_ALPHA * (imu.az_g - this->az_lpf_);
    }

    imu.gx_rad_s = std::clamp(std::isfinite(this->gx_lpf_) ? this->gx_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu.gy_rad_s = std::clamp(std::isfinite(this->gy_lpf_) ? this->gy_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);
    imu.gz_rad_s = std::clamp(std::isfinite(this->gz_lpf_) ? this->gz_lpf_ : 0.0f, -GYRO_LIMIT, GYRO_LIMIT);

    imu.ax_g = this->ax_lpf_;
    imu.ay_g = this->ay_lpf_;
    imu.az_g = this->az_lpf_;

    ctx.imu = imu;
    ctx.imu_valid = true;

    this->armed_imu_cache_ = imu;
    this->armed_imu_valid_ = true;

    return ESP_OK;
}

/*
 * 4. Estimator medium
 *
 * EKF는 계산량이 있으므로 기본 1000Hz로 시작한다.
 * 2kHz main loop에서는 두 번 중 한 번 실행된다.
 * 4kHz main loop에서는 네 번 중 한 번 실행된다.
 */

esp_err_t KSSDrone::ArmedEstimatorMedium(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    if (!ctx.imu_valid)
    {
        return ESP_OK;
    }

    if (!ctx.target_valid)
    {
        return ESP_OK;
    }

    float estimator_dt = 0.0f;
    if (!ConsumePeriodicDt(this->armed_estimator_dt_accum_,
                           dt,
                           ARMED_ESTIMATOR_PERIOD_SEC,
                           estimator_dt))
    {
        if (this->armed_state_valid_)
        {
            ctx.ekf_input = this->armed_ekf_input_cache_;
            ctx.ekf_out = this->armed_ekf_out_cache_;
            ctx.state = this->armed_state_cache_;

            ctx.ekf_valid = this->armed_ekf_valid_;
            ctx.state_valid = true;
        }

        return ESP_OK;
    }

    const IMU_PARESED_DATA& imu = ctx.imu;

    EKFAttitudeInput ekf_input{};

    ekf_input.ax = imu.ax_g;
    ekf_input.ay = imu.ay_g;
    ekf_input.az = imu.az_g;
    ekf_input.gx = imu.gx_rad_s;
    ekf_input.gy = imu.gy_rad_s;
    ekf_input.gz = imu.gz_rad_s;
    ekf_input.dt = estimator_dt;

    const float acc_norm = sqrtf(
        imu.ax_g * imu.ax_g +
        imu.ay_g * imu.ay_g +
        imu.az_g * imu.az_g
    );

    this->ekf_.Predict(ekf_input);

    const bool allow_acc_update =
        (!this->is_airborne_) &&
        (acc_norm >= IMU_WRONG_DATA_MIN) &&
        (acc_norm <= IMU_WRONG_DATA_MAX) &&
        (fabsf(imu.gx_rad_s) < 0.8f) &&
        (fabsf(imu.gy_rad_s) < 0.8f);

    if (allow_acc_update)
    {
        this->ekf_.Update(ekf_input);
    }

    EKFAttitudeOutput ekf_out = this->ekf_.GetOutput();

    AttitudeTarget target = ctx.target;

    if (ctx.cmd_valid && ctx.cmd != nullptr)
    {
        const ControlPacket& cmd = *ctx.cmd;

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
            target.yaw_rate_rad_s = 0.0f;
            target.yaw_hold_enable = true;
        }

        target.yaw_rad = this->yaw_hold_rad_;

        this->UpdateDynamicTakeoffTrim(
            target,
            ctx.throttle_rate_cmd
        );
    }

    AttitudeState state{};

    state.roll_rad =
        ekf_out.roll_rad
        - this->roll_offset_rad_
        - this->roll_trim_rad_
        - this->roll_manual_trim_rad_
        - this->takeoff_roll_trim_rad_
        - this->dynamic_roll_trim_rad_;

    state.pitch_rad =
        ekf_out.pitch_rad
        - this->pitch_offset_rad_
        - this->pitch_trim_rad_
        - this->pitch_manual_trim_rad_
        - this->takeoff_pitch_trim_rad_
        - this->dynamic_pitch_trim_rad_;

    state.yaw_rad = ekf_out.yaw_rad;

    state.gyro_x_rad_s = imu.gx_rad_s;
    state.gyro_y_rad_s = imu.gy_rad_s;
    state.gyro_z_rad_s = imu.gz_rad_s;

    ctx.ekf_input = ekf_input;
    ctx.ekf_out = ekf_out;
    ctx.state = state;

    ctx.ekf_valid = true;
    ctx.state_valid = true;
    ctx.estimator_updated = true;

    this->armed_ekf_input_cache_ = ekf_input;
    this->armed_ekf_out_cache_ = ekf_out;
    this->armed_state_cache_ = state;

    ctx.target = target;
    ctx.target_valid = true;

    this->armed_ekf_valid_ = true;
    this->armed_state_valid_ = true;

    return ESP_OK;
}

/*
 * 5. Airborne slow
 *
 * 기존 ArmedPrepareFast()에서 airborne 상태 계측 부분을 이동.
 * 기본 250Hz.
 */

void KSSDrone::ArmedAirborneSlow(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return;
    }

    if (!ctx.target_valid)
    {
        return;
    }

    float airborne_dt = 0.0f;
    if (!ConsumePeriodicDt(this->armed_airborne_dt_accum_,
                           dt,
                           ARMED_AIRBORNE_PERIOD_SEC,
                           airborne_dt))
    {
        return;
    }

    this->UpdateAirborneState(airborne_dt, ctx.target);

    ctx.airborne_updated = true;
}
/*
 * 6. Soft compensation slow
 *
 * 기존 0.004s 하드코딩을 제거하고 config 기반 250Hz로 이동.
 */

void KSSDrone::ArmedSlowCompUpdate(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return;
    }

    if (!ctx.cmd_valid || !ctx.imu_valid || !ctx.ekf_valid || !ctx.state_valid || !ctx.target_valid)
    {
        return;
    }

    float slow_dt = 0.0f;
    if (!ConsumePeriodicDt(this->armed_slow_comp_dt_accum_,
                           dt,
                           ARMED_SOFT_COMP_PERIOD_SEC,
                           slow_dt))
    {
        return;
    }

    this->UpdatePseudoVelocitySlow(slow_dt, ctx);
    this->UpdateVoltageCompSlow(ctx);
    this->UpdateAutoTrimSlow(slow_dt, ctx);
}
/*
 * 7. Control fast
 *
 * 매 loop 실행.
 *
 * 여기에는 PID / mixer / output만 남기는 것이 목표.
 * EKF는 ArmedEstimatorMedium()으로 이동.
 * command packet 직접 접근은 제거하고 armed_setpoint_를 사용한다.
 */

esp_err_t KSSDrone::ArmedControlFast(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    if (!ctx.target_valid || !ctx.state_valid || !ctx.imu_valid)
    {
        return ESP_OK;
    }

    /*
     * Rate PID는 최신 gyro를 사용한다.
     */
    ctx.state.gyro_x_rad_s = ctx.imu.gx_rad_s;
    ctx.state.gyro_y_rad_s = ctx.imu.gy_rad_s;
    ctx.state.gyro_z_rad_s = ctx.imu.gz_rad_s;

    /*
     * slow compensation은 state에만 적용.
     */
    ctx.state.pitch_rad -= this->cached_pitch_vel_comp_rad_;
    ctx.state.roll_rad  -= this->cached_roll_vel_comp_rad_;

    // const bool tilt_exceeded =
    //     (fabs(ctx.state.roll_rad) > TILT_LIMIT_RAD) ||
    //     (fabs(ctx.state.pitch_rad) > TILT_LIMIT_RAD);

    // if (tilt_exceeded)
    // {
    //     this->tilt_trigger_dt_ += dt;
    // }
    // else
    // {
    //     this->tilt_trigger_dt_ = 0.0f;
    // }

    // if (this->tilt_trigger_dt_ > TILT_TRIGGER_DT)
    // {
    //     this->debug_disarm_reason_ = DisarmReason::TILT;
    //     this->landing_mode_ = LandingMode::FAILSAFE;
    //     this->ChangeState(DroneState::LANDING);
    //     return ESP_OK;
    // }

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
     * 1kHz DShot 출력에서 사용할 최신 PID 결과 cache.
     */
    this->armed_pid_out_cache_ = ctx.pid_out;
    this->armed_pid_valid_ = true;

    return ESP_OK;
}

esp_err_t KSSDrone::ArmedOutputFast(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return ESP_OK;
    }

    if (!ctx.target_valid || !ctx.state_valid)
    {
        return ESP_OK;
    }

    /*
     * 1. PID output limiting
     */
    ctx.pid_out.throttle = std::clamp(ctx.pid_out.throttle, 0.0f, 1.0f);
    ctx.pid_out.roll     = std::clamp(ctx.pid_out.roll, -0.5f, 0.5f);
    ctx.pid_out.pitch    = std::clamp(ctx.pid_out.pitch, -0.5f, 0.5f);
    ctx.pid_out.yaw      = std::clamp(ctx.pid_out.yaw, -0.5f, 0.5f);

    /*
     * 2. Low throttle / pre-air authority
     */
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

    /*
     * 3. PID authority reduction at low throttle
     */
    const float roll_pitch_comp =
        PID_MIN_AUTHORITY_RP +
        (1.0f - PID_MIN_AUTHORITY_RP) * ctx.target.throttle;

    const float yaw_comp =
        0.85f + 0.15f * ctx.target.throttle;

    ctx.pid_out.roll  *= roll_pitch_comp;
    ctx.pid_out.pitch *= roll_pitch_comp;
    ctx.pid_out.yaw   *= yaw_comp;

    ctx.pid_out.roll  = std::clamp(ctx.pid_out.roll, -0.5f, 0.5f);
    ctx.pid_out.pitch = std::clamp(ctx.pid_out.pitch, -0.5f, 0.5f);
    ctx.pid_out.yaw   = std::clamp(ctx.pid_out.yaw, -0.5f, 0.5f);

    /*
     * 4. Yaw priority / hover boost
     * pid_out 기반이므로 fast에 유지.
     */
    const float rp_activity =
        std::max(std::fabs(ctx.pid_out.roll), std::fabs(ctx.pid_out.pitch));

    float yaw_protect_blend = std::clamp(
        (rp_activity - YAW_RP_PROTECT_START) /
        (YAW_RP_PROTECT_FULL - YAW_RP_PROTECT_START),
        0.0f,
        1.0f
    );

    /*
    * 5. Throttle PID Attenuation
    *
    * 높은 throttle에서 증가하는 모터 제어 권한으로 인한
    * Roll/Pitch/Yaw 진동을 줄이기 위해 PID axis 출력을 감쇠한다.
    *
    * Feed-forward 및 mixer bias에는 적용하지 않는다.
    */
    if (ctx.target.throttle > TPA_BREAKPOINT)
    {
        float tpa_ratio =
            (ctx.target.throttle - TPA_BREAKPOINT) /
            (1.0f - TPA_BREAKPOINT);

        tpa_ratio = std::clamp(
            tpa_ratio,
            0.0f,
            1.0f);

        const float rp_tpa = std::clamp(
            1.0f - TPA_RATE_RP * tpa_ratio,
            0.5f,
            1.0f);

        const float yaw_tpa = std::clamp(
            1.0f - TPA_RATE_YAW * tpa_ratio,
            0.5f,
            1.0f);

        ctx.pid_out.roll  *= rp_tpa;
        ctx.pid_out.pitch *= rp_tpa;
        ctx.pid_out.yaw   *= yaw_tpa;
    }

    yaw_protect_blend = SmoothStep01(yaw_protect_blend);

    float yaw_priority_scale =
        1.0f - YAW_RP_PROTECT_MAX_CUT * yaw_protect_blend;

    yaw_priority_scale = std::clamp(
        yaw_priority_scale,
        YAW_RP_PROTECT_MIN_SCALE,
        1.0f
    );

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

    ctx.pid_out.yaw *= yaw_hover_boost;
    ctx.pid_out.yaw *= yaw_priority_scale;
    ctx.pid_out.yaw = std::clamp(ctx.pid_out.yaw, -0.5f, 0.5f);

    /*
     * 6. Thrust curve
     */
    const float t = std::clamp(ctx.pid_out.throttle, 0.0f, 1.0f);
    const float expo = std::clamp(THRUST_EXPO, 0.0f, 1.0f);

    float thrust_cmd = t * t * (1.0f - expo) + t * expo;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    if (this->state_ == DroneState::ARMED)
    {
        thrust_cmd = std::max(thrust_cmd, IDLE_THROTTLE);
    }

    /*
     * 7. Cached output compensation
     */
    const ArmedOutputCache& oc = this->armed_output_cache_;

    thrust_cmd *= oc.thrust_v_scale;
    thrust_cmd *= oc.high_thr_scale;

    const float vz_damping =
        -this->vertical_velocity_est_ * K_VZ;

    thrust_cmd += vz_damping;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    /*
     * 8. Mixer input
     */
    MixerInput mix_in{};
    mix_in.throttle = thrust_cmd;
    mix_in.roll     = ctx.pid_out.roll  * oc.axis_rp_v_scale;
    mix_in.pitch    = ctx.pid_out.pitch * oc.axis_rp_v_scale;
    mix_in.yaw      = ctx.pid_out.yaw   * oc.axis_yaw_v_scale;

    /*
     * 9. VZ based attitude compensation
     */
    float vz_comp = 0.0f;

    if (this->is_airborne_)
    {
        vz_comp = std::clamp(this->vertical_velocity_est_, -1.0f, 1.0f);
    }

    mix_in.roll  += K_ROLL_VZ_COMP  * vz_comp;
    mix_in.pitch += K_PITCH_VZ_COMP * vz_comp;
    mix_in.yaw   += K_YAW_VZ_COMP   * vz_comp;

    /*
     * 10. Yaw throttle-down feed-forward
     */
    if (this->is_airborne_ && ctx.throttle_rate_cmd < 0.0f)
    {
        float yaw_throttle_down_ff =
            -ctx.throttle_rate_cmd * YAW_THR_DOWN_FF_GAIN;

        yaw_throttle_down_ff = std::clamp(
            yaw_throttle_down_ff,
            0.0f,
            YAW_THR_DOWN_FF_MAX
        );

        mix_in.yaw += yaw_throttle_down_ff;
    }

    if (this->is_airborne_)
    {
        mix_in.yaw += YAW_MIX_BIAS;
    }

    /*
     * 11. Mixer with cached motor multipliers
     */
    MixerOutput mix_out =
        MixMotorsWithMultiplier(
            mix_in,
            oc.fl_mul,
            oc.fr_mul,
            oc.rr_mul,
            oc.rl_mul
        );

    /*
     * 12. Cached ground blend
     */
    const float ground_blend = oc.ground_blend;

    mix_out.m1 = thrust_cmd + (mix_out.m1 - thrust_cmd) * ground_blend;
    mix_out.m2 = thrust_cmd + (mix_out.m2 - thrust_cmd) * ground_blend;
    mix_out.m3 = thrust_cmd + (mix_out.m3 - thrust_cmd) * ground_blend;
    mix_out.m4 = thrust_cmd + (mix_out.m4 - thrust_cmd) * ground_blend;

    /*
     * 13. 평균 thrust_cmd로 복귀
     */
    const float avg =
        0.25f * (mix_out.m1 + mix_out.m2 + mix_out.m3 + mix_out.m4);

    const float delta = thrust_cmd - avg;

    mix_out.m1 += delta;
    mix_out.m2 += delta;
    mix_out.m3 += delta;
    mix_out.m4 += delta;

    /*
     * 14. Yaw output bias
     */
    if (this->is_airborne_)
    {
        mix_out.m1 += YAW_OUTPUT_BIAS_AIRBORNE;
        mix_out.m2 -= YAW_OUTPUT_BIAS_AIRBORNE;
        mix_out.m3 += YAW_OUTPUT_BIAS_AIRBORNE;
        mix_out.m4 -= YAW_OUTPUT_BIAS_AIRBORNE;
    }

    /*
     * 15. Normalize
     */
    MixNormalize(mix_out, 0.0f, 1.0f);

    /*
     * 16. DSHOT conversion and motor output
     */
    uint16_t d1 = ThrottleToDshotWithIdle(mix_out.m1);
    uint16_t d2 = ThrottleToDshotWithIdle(mix_out.m2);
    uint16_t d3 = ThrottleToDshotWithIdle(mix_out.m3);
    uint16_t d4 = ThrottleToDshotWithIdle(mix_out.m4);

    d1 = std::max<uint16_t>(d1, IDLE_DSHOT_THRESHOLD);
    d2 = std::max<uint16_t>(d2, IDLE_DSHOT_THRESHOLD);
    d3 = std::max<uint16_t>(d3, IDLE_DSHOT_THRESHOLD);
    d4 = std::max<uint16_t>(d4, IDLE_DSHOT_THRESHOLD);

    esp_err_t ret =
        this->motor_interface_.SetMotorOutput(d1, d2, d3, d4);

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "RMT - Motor Control failed.");
        return ret;
    }


    this->armed_log_dt_accum_ += dt;

    if (this->armed_log_dt_accum_ >= 0.004f)
    {
        this->armed_log_dt_accum_ = 0.0f;


        /*
        * 17. Telemetry
        */
        this->tpkt_.roll_rad = ctx.state.roll_rad;
        this->tpkt_.pitch_rad = ctx.state.pitch_rad;

        this->tpkt_.gyro_x_rad_s = ctx.imu.gx_rad_s;
        this->tpkt_.gyro_y_rad_s = ctx.imu.gy_rad_s;
        this->tpkt_.gyro_z_rad_s = ctx.imu.gz_rad_s;

        this->tpkt_.throttle = ctx.target.throttle;
        this->tpkt_.debug_disarm_reason = this->debug_disarm_reason_;


        /*
        * 18. Log update, 250Hz
        */
        this->log_.dt = dt;
        this->log_.roll_rad = ctx.state.roll_rad;
        this->log_.pitch_rad = ctx.state.pitch_rad;

        this->log_.gyro_x_rad_s = ctx.imu.gx_rad_s;
        this->log_.gyro_y_rad_s = ctx.imu.gy_rad_s;
        this->log_.gyro_z_rad_s = ctx.imu.gz_rad_s;

        this->log_.ax_g = ctx.imu.ax_g;
        this->log_.ay_g = ctx.imu.ay_g;
        this->log_.az_g = ctx.imu.az_g;

        this->log_.throttle_cmd =
            (ctx.cmd_valid && ctx.cmd != nullptr)
                ? ctx.cmd->throttle
                : ctx.target.throttle;

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
    }

    return ESP_OK;
}

void KSSDrone::ArmedOutputCompMedium(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return;
    }

    if (!ctx.target_valid)
    {
        return;
    }

    float comp_dt = 0.0f;
    if (!ConsumePeriodicDt(this->armed_output_comp_dt_accum_,
                           dt,
                           ARMED_OUTPUT_COMP_PERIOD_SEC,
                           comp_dt))
    {
        return;
    }

    (void)comp_dt;

    /*
     * pid_out.throttle 기준.
     * 이 함수는 ArmedControlFast() 이후에 호출되어야 한다.
     */
    const float t = std::clamp(
        this->armed_pid_valid_
            ? this->armed_pid_out_cache_.throttle
            : ctx.target.throttle,
        0.0f,
        1.0f
    );

    /*
     * 1. Voltage compensation scale
     */
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

    this->armed_output_cache_.thrust_v_scale =
        1.0f + (this->cached_thrust_v_scale_ - 1.0f) * volt_comp_blend;

    this->armed_output_cache_.axis_rp_v_scale =
        1.0f + (this->cached_axis_rp_v_scale_ - 1.0f) * volt_comp_blend;

    this->armed_output_cache_.axis_yaw_v_scale =
        1.0f + (this->cached_axis_yaw_v_scale_ - 1.0f) * volt_comp_blend;

    /*
     * 2. High throttle output limiting
     */
    float high_thr_blend = std::clamp(
        (t - HIGH_THR_LIMIT_START) /
        (HIGH_THR_LIMIT_FULL - HIGH_THR_LIMIT_START),
        0.0f,
        1.0f
    );

    high_thr_blend = SmoothStep01(high_thr_blend);

    this->armed_output_cache_.high_thr_scale =
        1.0f - HIGH_THR_MAX_CUT * high_thr_blend;

    this->armed_output_cache_.high_thr_scale =
        std::clamp(this->armed_output_cache_.high_thr_scale, 0.0f, 1.0f);

    /*
     * 3. Ground blend
     */
    if (!this->is_airborne_)
    {
        float ground_throttle = std::clamp(
            (ctx.target.throttle - IDLE_THROTTLE) / GROUND_BLEND_RANGE,
            0.0f,
            1.0f
        );

        float ground_blend = SmoothStep01(ground_throttle);

        this->armed_output_cache_.ground_blend =
            0.15f + 0.85f * ground_blend;
    }
    else
    {
        this->armed_output_cache_.ground_blend = 1.0f;
    }

    /*
     * 4. Motor bias multiplier
     */
    float front_bias = 1.0f;
    float rear_bias  = 1.0f;
    float left_bias  = 1.0f;
    float right_bias = 1.0f;

    if (this->is_airborne_)
    {
        constexpr float hover_throttle = 0.52f;

        const float h_dt = t - hover_throttle;
        const float k_gain = std::clamp(h_dt, -0.3f, 0.3f);

        const float h_gain = 0.00f;

        front_bias = FRONT_THRUST_BIAS + h_gain * k_gain;
        rear_bias  = REAR_THRUST_BIAS  - h_gain * k_gain;

        left_bias  = LEFT_THRUST_BIAS;
        right_bias = RIGHT_THRUST_BIAS;
    }

    this->armed_output_cache_.fl_mul =
        M1_FL_BIAS * front_bias * left_bias;

    this->armed_output_cache_.fr_mul =
        M2_FR_BIAS * front_bias * right_bias;

    this->armed_output_cache_.rr_mul =
        M3_RR_BIAS * rear_bias * right_bias;

    this->armed_output_cache_.rl_mul =
        M4_RL_BIAS * rear_bias * left_bias;
}

void KSSDrone::ArmedTiltSafetyMedium(const float dt, ArmedContext& ctx)
{
    if (this->state_ != DroneState::ARMED)
    {
        return;
    }

    if (!ctx.state_valid)
    {
        return;
    }

    if (this->drone_mode_ != DroneMode::ANGLE_SELF_LEVEL)
    {
        this->tilt_trigger_dt_ = 0.0f;
        return;
    }

    const bool tilt_exceeded =
        (std::fabs(ctx.state.roll_rad) > TILT_LIMIT_RAD) ||
        (std::fabs(ctx.state.pitch_rad) > TILT_LIMIT_RAD);

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
    }
}