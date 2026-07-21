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

    if (this->telemetry_send_dt_ >= CHECK_SEND_TELEMETRY_MS && rx_dt_us > 5000)
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
