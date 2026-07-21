#include "kss_drone.h"

static const char* TAG = "kss_drone";

esp_err_t KSSDrone::BackgroundJobs(const float dt)
{
    esp_err_t ret = this->FastBackgroundJobs(dt);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return this->SlowBackgroundJobs(dt);
}

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

esp_err_t KSSDrone::SlowBackgroundJobs(const float dt)
{
    this->battery_check_dt_ += dt;

    if (this->battery_check_dt_ >= CHECK_BATTERY_PERIOD_S)
    {
        while (this->battery_check_dt_ >=
            CHECK_BATTERY_PERIOD_S)
        {
            this->battery_check_dt_ -=
                CHECK_BATTERY_PERIOD_S;
        }

        const esp_err_t ret =
            this->battery_monitor_.Update(
                CHECK_BATTERY_PERIOD_S);

        if (ret != ESP_OK)
        {
            return ret;
        }

        this->tpkt_.battery_voltage =
            this->battery_monitor_.GetVoltage();

        this->tpkt_.battery_percent =
            this->battery_monitor_.GetPercent();
    }
    // const int64_t rx_dt_us =
    //     esp_timer_get_time() - this->esp_now_interface_.GetLastRxTimeUs();

    // const int64_t rx_dt_us =
    //     esp_timer_get_time() - this->crsf_receiver_.GetLastRxTimeUs();

    this->telemetry_send_dt_ += dt;

    // if (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_MS && rx_dt_us > 2000)
    if (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_MS)
    {
        while (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_MS)
        {
            this->telemetry_send_dt_ -= CHECK_SEND_TELEMETRY_MS;
        }

        this->tpkt_.mode = static_cast<uint8_t>(this->drone_mode_);
        this->tpkt_.state = static_cast<uint8_t>(this->state_);

        const TelemetryPacket send_pkt_ = this->tpkt_;

        // esp_err_t ret = this->esp_now_interface_.SendTelemetry(send_pkt_);
        esp_err_t ret = this->crsf_receiver_.SendTelemetry(send_pkt_);

        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG,
                    "CRSF telemetry send failed: %s",
                    esp_err_to_name(ret));
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
        const bool flight_active =
            this->state_ == DroneState::ARMED ||
            this->state_ == DroneState::LANDING;

        const bool likely_airborne =
            flight_active &&
            (this->is_airborne_ ||
            this->throttle_prev_ >
                (IDLE_THROTTLE + 0.05f) ||
            this->landing_throttle_ >
                (IDLE_THROTTLE + 0.05f));

        if (likely_airborne)
        {
            ESP_LOGW(
                TAG,
                "DISARM_REQUEST rejected in airborne");
            return;
        }

        this->armed_ = false;
        return;
    }

    return;
}
