#include "kss_drone.h"

static const char* TAG = "kss_drone";

esp_err_t KSSDrone::FastBackgroundJobs(const float dt)
{
    (void)dt;

    /*
     * 명령 cache / edge event는 안전 관련이므로 일단 1kHz 유지.
     * DISARM, SOFT_LANDING, EMERGENCY 계열 반응성을 유지하기 위함.
     */
    this->UpdateCommandCache();
    this->HandleCommandEvents();

    return ESP_OK;
}

// Called on the existing 10ms slow-background cadence, after flight control.
// All per-job elapsed time accumulators advance even if the job is deferred.
// ARMED/LANDING admission uses the remaining time before the next 500us slot;
// other states retain unconditional background service.
esp_err_t KSSDrone::ProcessSlowBackground(const float dt,
                                          const int64_t deadline_us)
{
    this->battery_check_dt_ += dt;
    this->telemetry_send_dt_ += dt;
    this->led_update_dt_ += dt;

#if ENABLE_CMD_DETAIL_LOG
    if (this->state_ == DroneState::ARMED ||
        this->state_ == DroneState::LANDING)
    {
        this->log_send_dt_ += dt;
    }
    else
    {
        this->log_send_dt_ = 0.0f;
    }
#endif

#if MAIN_LOOP_RATE_HZ > 1000
    const bool budgeted =
        this->state_ == DroneState::ARMED ||
        this->state_ == DroneState::LANDING;
#else
    (void)deadline_us;
    const bool budgeted = false;
#endif

    const auto fits = [budgeted, deadline_us](const int64_t budget_us)
    {
        return !budgeted ||
            deadline_us - esp_timer_get_time() >= budget_us;
    };

    // Battery remains 8 consecutive ADC samples + block average + LPF.
    // Its 400-600us cost rarely fits a 500us flight-control slot, so it
    // has a bounded deferral: force service by ~70ms of accumulated time.
    if (this->battery_check_dt_ >= CHECK_BATTERY_PERIOD_S)
    {
        const bool forced = budgeted &&
            this->battery_check_dt_ >= BG_BATTERY_MAX_GAP_S;
        if (forced || fits(BG_BATTERY_BUDGET_US))
        {
            const float actual_dt = this->battery_check_dt_;
            const esp_err_t ret = this->BatteryBackgroundJob(actual_dt);
            if (ret != ESP_OK)
            {
                return ret;
            }
            this->battery_check_dt_ = 0.0f;
            this->bg_schedule_stats_.battery_forced += forced ? 1U : 0U;
            this->bg_schedule_stats_.battery_max_gap_us =
                std::max(this->bg_schedule_stats_.battery_max_gap_us,
                         static_cast<uint32_t>(actual_dt * 1000000.0f));

            // Avoid stacking telemetry or LED onto an already long slot.
            if (budgeted)
            {
                return ESP_OK;
            }
        }
        else
        {
            ++this->bg_schedule_stats_.battery_deferred;
        }
    }

    if (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_S)
    {
        const bool forced = budgeted &&
            this->telemetry_send_dt_ >= BG_TELEMETRY_MAX_GAP_S;
        if (forced || fits(BG_TELEMETRY_BUDGET_US))
        {
            const float actual_dt = this->telemetry_send_dt_;
            const esp_err_t ret = this->TelemetryBackgroundJob();
            if (ret != ESP_OK)
            {
                return ret;
            }
            this->telemetry_send_dt_ = 0.0f;
            this->bg_schedule_stats_.telemetry_forced += forced ? 1U : 0U;
            this->bg_schedule_stats_.telemetry_max_gap_us =
                std::max(this->bg_schedule_stats_.telemetry_max_gap_us,
                         static_cast<uint32_t>(actual_dt * 1000000.0f));

            if (budgeted)
            {
                return ESP_OK;
            }
        }
        else
        {
            ++this->bg_schedule_stats_.telemetry_deferred;
        }
    }

    if (this->led_update_dt_ >= BACKGROUND_SLOW_JOB_TIME)
    {
        const bool forced = budgeted &&
            this->led_update_dt_ >= BG_LED_MAX_GAP_S;
        if (forced || fits(BG_LED_BUDGET_US))
        {
            const float actual_dt = this->led_update_dt_;
            this->LedBackgroundJob(actual_dt);
            this->led_update_dt_ = 0.0f;
            this->bg_schedule_stats_.led_forced += forced ? 1U : 0U;
            this->bg_schedule_stats_.led_max_gap_us =
                std::max(this->bg_schedule_stats_.led_max_gap_us,
                         static_cast<uint32_t>(actual_dt * 1000000.0f));
        }
        else
        {
            ++this->bg_schedule_stats_.led_deferred;
        }
    }

#if ENABLE_CMD_DETAIL_LOG
    if (this->log_send_dt_ >= CHECK_SEND_LOG_S &&
        (this->state_ == DroneState::ARMED ||
         this->state_ == DroneState::LANDING))
    {
        if (fits(BG_DETAIL_LOG_BUDGET_US) ||
            this->log_send_dt_ >= BG_DETAIL_LOG_MAX_GAP_S)
        {
            this->DetailLogBackgroundJob();
            this->log_send_dt_ = 0.0f;
        }
    }
#endif

    return ESP_OK;
}

esp_err_t KSSDrone::BatteryBackgroundJob(const float actual_dt)
{
#if ENABLE_DISARM_TIMING_PROFILE
    const bool profile = this->state_ == DroneState::DISARMED;
    const uint32_t begin = profile
        ? static_cast<uint32_t>(esp_cpu_get_cycle_count()) : 0U;
#endif
    const esp_err_t ret = this->battery_monitor_.Update(actual_dt);
#if ENABLE_DISARM_TIMING_PROFILE
    if (profile)
    {
        this->disarm_timing_.Record(
            DisarmTimingStage::BG_BATTERY, begin, true);
    }
#endif
    if (ret != ESP_OK)
    {
        return ret;
    }

    this->tpkt_.battery_voltage = this->battery_monitor_.GetVoltage();
    this->tpkt_.battery_percent = this->battery_monitor_.GetPercent();
    return ESP_OK;
}

esp_err_t KSSDrone::TelemetryBackgroundJob()
{
    this->tpkt_.mode = static_cast<uint8_t>(this->drone_mode_);
    this->tpkt_.state = static_cast<uint8_t>(this->state_);
    const TelemetryPacket packet = this->tpkt_;

#if ENABLE_DISARM_TIMING_PROFILE
    const bool profile = this->state_ == DroneState::DISARMED;
    const uint32_t begin = profile
        ? static_cast<uint32_t>(esp_cpu_get_cycle_count()) : 0U;
#endif
    const esp_err_t ret = this->crsf_receiver_.SendTelemetry(packet);
#if ENABLE_DISARM_TIMING_PROFILE
    if (profile)
    {
        this->disarm_timing_.Record(
            DisarmTimingStage::BG_TELEMETRY, begin, true);
    }
#endif
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "CRSF telemetry send failed: %s",
                 esp_err_to_name(ret));
    }
    // Match previous behavior: telemetry errors are reported, not fatal.
    return ESP_OK;
}

void KSSDrone::LedBackgroundJob(const float actual_dt)
{
#if ENABLE_DISARM_TIMING_PROFILE
    const bool profile = this->state_ == DroneState::DISARMED;
    const uint32_t begin = profile
        ? static_cast<uint32_t>(esp_cpu_get_cycle_count()) : 0U;
#endif
    this->led_controller_.Update(actual_dt);
#if ENABLE_DISARM_TIMING_PROFILE
    if (profile)
    {
        this->disarm_timing_.Record(
            DisarmTimingStage::BG_LED, begin, true);
    }
#endif
}

void KSSDrone::DetailLogBackgroundJob()
{
#if ENABLE_CMD_DETAIL_LOG
    ESP_LOGI(TAG,
             "yaw=%.3f thr_cmd=%.3f thr_used=%.3f gyro_z=%.3f state=%u",
             this->log_.yaw_out,
             this->log_.throttle_cmd,
             this->log_.throttle_used,
             this->log_.gyro_z_rad_s,
             static_cast<unsigned>(this->log_.state));
#endif
}

void KSSDrone::UpdateCommandCache()
{
    ControlPacket cmd{};

    if (!this->crsf_receiver_.GetLatestCommand(cmd))
    {
        this->active_cmd_valid_ = false;
        return;
    }

    const bool new_command =
        !this->control_seq_ready_ ||
        SeqDiff16(
            cmd.seq,
            this->last_control_seq_) > 0;

    if (!new_command)
    {
        /*
         * 같은 CRSF snapshot을 1kHz에서 다시 읽은 정상 상황.
         */
        return;
    }

    this->last_control_seq_ = cmd.seq;
    this->control_seq_ready_ = true;

    this->active_cmd_ = cmd;
    this->active_cmd_valid_ = true;
}

void KSSDrone::HandleCommandEvents()
{
    if (!this->active_cmd_valid_)
    {
        return;
    }

    const ControlPacket cmd = this->active_cmd_;

    if (cmd.cmd_flags & CMD_EMERGENCY_STOP)
    {
        /*
        * 안전 출력은 중복 여부와 관계없이 항상 적용
        */
        this->armed_ = false;
        this->landing_throttle_ = 0.0f;
        this->throttle_prev_ = 0.0f;

        this->pid_controller_.ResetIntegrator();
        this->motor_interface_.SetMotorOutput(0, 0, 0, 0);

        const bool new_emergency =
            !this->command_seq_ready_ ||
            cmd.cmd_seq != this->last_command_seq_;

        if (new_emergency)
        {
            this->command_seq_ready_ = true;
            this->last_command_seq_ = cmd.cmd_seq;

            ESP_LOGE(TAG, "EMERGENCY STOP!");
            this->ChangeState(DroneState::DISARMED);
        }

        return;
    }

    if (cmd.mode <= static_cast<uint8_t>(
                        DroneMode::ANGLE_SELF_LEVEL) &&
        this->state_ == DroneState::DISARMED)
    {
        const DroneMode new_mode =
            static_cast<DroneMode>(cmd.mode);

        if (new_mode != this->drone_mode_)
        {
            const DroneMode old_mode = this->drone_mode_;

            this->pid_controller_.Reset();
            this->throttle_prev_ = 0.0f;
            this->output_saturated_ = false;
            this->drone_mode_ = new_mode;

            ESP_LOGI(
                TAG,
                "Drone Mode Changed: %u -> %u",
                static_cast<unsigned>(old_mode),
                static_cast<unsigned>(new_mode));
        }
    }


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

    if (cmd.cmd_flags & CMD_ARM_REQUEST)
    {
        const bool safe_to_arm =
            this->state_ == DroneState::DISARMED &&
            this->imu_bias_ready_ &&
            this->ekf_ready_ &&
            this->disarmed_settle_dt_ >= DISARM_SETTLE_TIME &&
            cmd.throttle < 0.05f &&
            !this->armed_;

        if (safe_to_arm)
        {
            this->armed_ = true;
            this->pid_controller_.Reset();
            this->throttle_prev_ = 0.0f;
            this->landing_throttle_ = 0.0f;
            this->output_saturated_ = false;

            ESP_LOGI(TAG, "ARM_REQUEST accepted");

            this->ChangeState(DroneState::ARMING);
        }
        else
        {
            ESP_LOGW(
                TAG,
                "ARM rejected: state=%u imu=%d ekf=%d "
                "settle=%.3f/%.3f thr=%.3f armed=%d",
                static_cast<unsigned>(this->state_),
                static_cast<int>(this->imu_bias_ready_),
                static_cast<int>(this->ekf_ready_),
                this->disarmed_settle_dt_,
                DISARM_SETTLE_TIME,
                cmd.throttle,
                static_cast<int>(this->armed_));
        }

        return;
    }

    if (cmd.cmd_flags & CMD_DISARM_REQUEST)
    {
        this->armed_ = false;
        this->landing_throttle_ = 0.0f;
        this->throttle_prev_ = 0.0f;
        this->output_saturated_ = false;

        this->pid_controller_.ResetIntegrator();
        this->motor_interface_.SetMotorOutput(0, 0, 0, 0);

        ESP_LOGW(TAG, "DISARM_REQUEST accepted");

        this->ChangeState(DroneState::DISARMED);
        return;
    }

    return;
}
