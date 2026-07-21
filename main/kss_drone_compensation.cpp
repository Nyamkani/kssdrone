#include "kss_drone.h"

void KSSDrone::UpdateVoltageCompSlow(ArmedContext& ctx)
{
    (void)ctx;

    const float voltage = this->battery_monitor_.GetVoltage();

    float v_scale = 1.0f;

    if (voltage > BATTERY_VOLTAGE_MIN_VALID)
    {
        v_scale = BATTERY_VOLTAGE_REF / voltage;
        v_scale = std::clamp(v_scale, 1.0f, V_SCALE_MAX);
    }

    this->cached_thrust_v_scale_ =
        1.0f + (v_scale - 1.0f) * THRUST_VOLT_COMP_GAIN;

    this->cached_axis_rp_v_scale_ =
        1.0f + (v_scale - 1.0f) * AXIS_VOLT_COMP_GAIN_RP;

    this->cached_axis_yaw_v_scale_ =
        1.0f + (v_scale - 1.0f) * AXIS_VOLT_COMP_GAIN_YAW;
}

void KSSDrone::UpdateAutoTrimSlow(const float dt, ArmedContext& ctx)
{
    if (ctx.cmd == nullptr)
    {
        this->auto_roll_trim_dt_ = 0.0f;
        this->auto_roll_trim_accum_ = 0.0f;
        this->auto_pitch_trim_dt_ = 0.0f;
        this->auto_pitch_trim_accum_ = 0.0f;
        return;
    }

    /*
     * Auto trim은 느린 보정 기능이다.
     * dt는 ArmedSlowCompUpdate()에서 누적된 slow_dt를 넘긴다.
     *
     * 조건:
     * - airborne
     * - angle self-level mode
     * - 사용자가 roll/pitch stick을 크게 넣지 않음
     * - gyro rate가 충분히 작음
     * - throttle이 어느 정도 있음
     */
    const bool common_allow =
        this->is_airborne_ &&
        ctx.target.mode == DroneMode::ANGLE_SELF_LEVEL &&
        std::fabs(ctx.target.roll_rad) < DEG2RAD(3.0f) &&
        std::fabs(ctx.target.pitch_rad) < DEG2RAD(3.0f) &&
        std::fabs(ctx.state.gyro_x_rad_s) < DEG2RAD(30.0f) &&
        std::fabs(ctx.state.gyro_y_rad_s) < DEG2RAD(30.0f) &&
        std::fabs(ctx.state.gyro_z_rad_s) < DEG2RAD(30.0f) &&
        ctx.target.throttle > 0.35f;

    if (!common_allow)
    {
        this->auto_roll_trim_dt_ = 0.0f;
        this->auto_roll_trim_accum_ = 0.0f;

        this->auto_pitch_trim_dt_ = 0.0f;
        this->auto_pitch_trim_accum_ = 0.0f;

        return;
    }

    /*
     * 현재 구조에서는 ArmedControlFast()가 이 함수 이후에 호출되므로,
     * 아래 error는 직전 PID update의 error일 수 있다.
     * Auto trim은 긴 시간 평균이므로 1-loop 지연은 허용 가능하다.
     */
    const float roll_error =
        this->pid_controller_.GetRollAngleLastError();

    const float pitch_error =
        this->pid_controller_.GetPitchAngleLastError();

    if (std::isfinite(roll_error))
    {
        this->auto_roll_trim_accum_ += roll_error * dt;
        this->auto_roll_trim_dt_ += dt;
    }
    else
    {
        this->auto_roll_trim_accum_ = 0.0f;
        this->auto_roll_trim_dt_ = 0.0f;
    }

    if (std::isfinite(pitch_error))
    {
        this->auto_pitch_trim_accum_ += pitch_error * dt;
        this->auto_pitch_trim_dt_ += dt;
    }
    else
    {
        this->auto_pitch_trim_accum_ = 0.0f;
        this->auto_pitch_trim_dt_ = 0.0f;
    }

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

        this->roll_trim_rad_ = std::clamp(
            this->roll_trim_rad_,
            -DEG2RAD(3.0f),
            DEG2RAD(3.0f)
        );

        this->auto_roll_trim_dt_ = 0.0f;
        this->auto_roll_trim_accum_ = 0.0f;
    }

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

        this->pitch_trim_rad_ = std::clamp(
            this->pitch_trim_rad_,
            -DEG2RAD(3.0f),
            DEG2RAD(3.0f)
        );

        this->auto_pitch_trim_dt_ = 0.0f;
        this->auto_pitch_trim_accum_ = 0.0f;
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

void KSSDrone::UpdatePseudoVelocitySlow(const float dt, ArmedContext& ctx)
{
    const float roll_phys  = ctx.ekf_out.roll_rad  - this->roll_offset_rad_;
    const float pitch_phys = ctx.ekf_out.pitch_rad - this->pitch_offset_rad_;
    const float yaw_phys   = ctx.ekf_out.yaw_rad;

    const float cr = cosf(roll_phys);
    const float sr = sinf(roll_phys);

    const float cp = cosf(pitch_phys);
    const float sp = sinf(pitch_phys);

    const float cy = cosf(yaw_phys);
    const float sy = sinf(yaw_phys);

    const float ax_b = ctx.imu.ax_g;
    const float ay_b = ctx.imu.ay_g;
    const float az_b = ctx.imu.az_g;

    const float ax_world =
        (cy * cp) * ax_b +
        (cy * sp * sr - sy * cr) * ay_b +
        (cy * sp * cr + sy * sr) * az_b;

    const float ay_world =
        (sy * cp) * ax_b +
        (sy * sp * sr + cy * cr) * ay_b +
        (sy * sp * cr - cy * sr) * az_b;

    float az_world =
        (-sp) * ax_b +
        (cp * sr) * ay_b +
        (cp * cr) * az_b;

    az_world -= 1.0f;

    this->ax_world_lpf_ =
        this->ax_world_lpf_ * (1.0f - AXY_LPF_ALPHA) +
        ax_world * AXY_LPF_ALPHA;

    this->ay_world_lpf_ =
        this->ay_world_lpf_ * (1.0f - AXY_LPF_ALPHA) +
        ay_world * AXY_LPF_ALPHA;

    this->vertical_accel_lpf_ =
        this->vertical_accel_lpf_ * (1.0f - AZ_LPF_ALPHA) +
        az_world * AZ_LPF_ALPHA;

    this->vx_est_ += this->ax_world_lpf_ * dt;
    this->vy_est_ += this->ay_world_lpf_ * dt;
    this->vertical_velocity_est_ += this->vertical_accel_lpf_ * dt;

    const float leak_xy = std::clamp(
        1.0f - VXY_LEAK_PER_SEC * dt,
        0.0f,
        1.0f
    );

    const float leak_z = std::clamp(
        1.0f - VZ_LEAK_PER_SEC * dt,
        0.0f,
        1.0f
    );

    this->vx_est_ *= leak_xy;
    this->vy_est_ *= leak_xy;
    this->vertical_velocity_est_ *= leak_z;

    this->vx_est_ = std::clamp(this->vx_est_, -1.0f, 1.0f);
    this->vy_est_ = std::clamp(this->vy_est_, -1.0f, 1.0f);

    this->vertical_velocity_est_ =
        std::clamp(this->vertical_velocity_est_, -2.0f, 2.0f);

    const ControlPacket& cmd = *ctx.cmd;

    const bool rp_stick_active =
        std::fabs(cmd.roll_rad)  > DEG2RAD(3.0f) ||
        std::fabs(cmd.pitch_rad) > DEG2RAD(3.0f);

    if (this->is_airborne_ && !rp_stick_active)
    {
        this->cached_pitch_vel_comp_rad_ =
            std::clamp(K_VX_TO_PITCH * this->vx_est_,
                       -VXY_COMP_MAX,
                       VXY_COMP_MAX);

        this->cached_roll_vel_comp_rad_ =
            std::clamp(K_VY_TO_ROLL * this->vy_est_,
                       -VXY_COMP_MAX,
                       VXY_COMP_MAX);
    }
    else
    {
        this->vx_est_ *= 0.98f;
        this->vy_est_ *= 0.98f;

        this->cached_pitch_vel_comp_rad_ = 0.0f;
        this->cached_roll_vel_comp_rad_ = 0.0f;
    }
}

