#include "kss_crsf_receiver.h"

static const char* TAG = "KSS_CRSF";

esp_err_t KssCrsfReceiver::Initialize(
    const uart_port_t uart_num,
    const QueueHandle_t uart_event_queue)
{
    /*
     * 이미 초기화했거나 태스크가 실행 중이면 거절
     */
    if (this->initialized_ || this->task_handle_ != nullptr)
    {
        ESP_LOGW(TAG, "CRSF receiver already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (uart_event_queue == nullptr)
    {
        ESP_LOGE(TAG,
                 "Initialize failed: UART event queue is null");
        return ESP_ERR_INVALID_ARG;
    }

    this->uart_num_ = uart_num;
    this->uart_event_queue_ = uart_event_queue;

    this->stop_requested_.store(
        false,
        std::memory_order_relaxed);

    this->rx_frame_count_.store(
        0,
        std::memory_order_relaxed);

    this->rc_frame_count_.store(
        0,
        std::memory_order_relaxed);

    this->crc_error_count_.store(
        0,
        std::memory_order_relaxed);

    this->frame_error_count_.store(
        0,
        std::memory_order_relaxed);

    this->last_rc_time_us_.store(
        0,
        std::memory_order_relaxed);

    this->seq_ = 0;
    this->cmd_seq_ = 0;
    this->telemetry_slot_ = 0;

    this->pending_cmd_flags_ =
        static_cast<uint8_t>(CMD_NONE);

    this->command_repeat_remaining_ = 0;

    this->switch_state_ready_ = false;
    this->arm_low_seen_ = false;

    this->prev_arm_switch_ = false;
    this->prev_mode_switch_ = false;
    this->prev_kill_switch_ = false;

    this->ResetParser();

    /*
     * StartTask()가 initialized_를 검사하므로
     * 태스크 생성 직전에 true로 설정한다.
     */
    this->initialized_ = true;

    ESP_LOGI(TAG,
             "CRSF receiver initialized on UART%d",
             static_cast<int>(this->uart_num_));

    const esp_err_t ret = this->StartTask();

    if (ret != ESP_OK)
    {
        /*
         * 태스크 생성 실패 시 Initialize() 재시도 가능하게 복구
         */
        this->initialized_ = false;
        this->uart_event_queue_ = nullptr;
        return ret;
    }

    return ESP_OK;
}

esp_err_t KssCrsfReceiver::StartTask()
{
    if (!this->initialized_)
    {
        ESP_LOGE(TAG, "StartTask failed: not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (this->task_handle_ != nullptr)
    {
        ESP_LOGW(TAG, "CRSF task already started");
        return ESP_OK;
    }

    this->stop_requested_.store(false, std::memory_order_release);

    const BaseType_t ret = xTaskCreatePinnedToCore(
        &KssCrsfReceiver::MainTask,
        "crsf_rx",
        4096,
        this,
        5,
        &this->task_handle_,
        0
    );

    if (ret != pdPASS)
    {
        this->task_handle_ = nullptr;
        ESP_LOGE(TAG, "Failed to create crsf_rx task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CRSF receiver task started");

    return ESP_OK;
}

void KssCrsfReceiver::StopTaskRequest()
{
    this->stop_requested_.store(true, std::memory_order_release);
}

void KssCrsfReceiver::MainTask(void* arg)
{
    auto* self = static_cast<KssCrsfReceiver*>(arg);
    self->MainLoop();
}

void KssCrsfReceiver::MainLoop()
{
    uart_event_t event{};
    uint8_t rx_buf[128]{};

    while (!this->stop_requested_.load(std::memory_order_acquire))
    {
        const BaseType_t received = xQueueReceive(
            this->uart_event_queue_,
            &event,
            pdMS_TO_TICKS(100)
        );

        if (received != pdTRUE)
        {
            continue;
        }

        switch (event.type)
        {
            case UART_DATA:
            {
                size_t remaining =
                    static_cast<size_t>(event.size);

                while (remaining > 0)
                {
                    const size_t request_size =
                        std::min(remaining, sizeof(rx_buf));

                    const int read_len = uart_read_bytes(
                        this->uart_num_,
                        rx_buf,
                        request_size,
                        pdMS_TO_TICKS(2)
                    );

                    if (read_len <= 0)
                    {
                        break;
                    }

                    for (int i = 0; i < read_len; ++i)
                    {
                        this->ProcessByte(rx_buf[i]);
                    }

                    remaining -= static_cast<size_t>(read_len);
                }

                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
            {
                ESP_LOGE(
                    TAG,
                    "UART RX overflow: type=%d",
                    static_cast<int>(event.type)
                );

                uart_flush_input(this->uart_num_);
                xQueueReset(this->uart_event_queue_);

                this->ResetParser();

                this->frame_error_count_.fetch_add(
                    1,
                    std::memory_order_relaxed
                );

                break;
            }

            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
            {
                ESP_LOGW(
                    TAG,
                    "UART frame/parity error: type=%d",
                    static_cast<int>(event.type)
                );

                this->ResetParser();

                this->frame_error_count_.fetch_add(
                    1,
                    std::memory_order_relaxed
                );

                break;
            }

            case UART_BREAK:
            {
                this->ResetParser();
                break;
            }

            default:
                break;
        }
    }

    ESP_LOGW(TAG, "CRSF receiver task stopped");

    this->task_handle_ = nullptr;
    vTaskDelete(nullptr);
}


static inline float ClampFloat(float v, float lo, float hi)
{
    return std::max(lo, std::min(v, hi));
}

static float CrsfRawToNorm(uint16_t raw)
{
    const float r = static_cast<float>(raw);

    if (r >= CRSF_RAW_MID)
    {
        return ClampFloat((r - CRSF_RAW_MID) /
                          (CRSF_RAW_MAX - CRSF_RAW_MID),
                          0.0f,
                          1.0f);
    }

    return ClampFloat((r - CRSF_RAW_MID) /
                      (CRSF_RAW_MID - CRSF_RAW_MIN),
                      -1.0f,
                      0.0f);
}

static float CrsfRawToThrottle01(uint16_t raw)
{
    const float r = static_cast<float>(raw);

    return ClampFloat((r - CRSF_RAW_MIN) /
                      (CRSF_RAW_MAX - CRSF_RAW_MIN),
                      0.0f,
                      1.0f);
}

static bool CrsfSwitchHigh(uint16_t raw)
{
    /*
     * 초기값.
     * 2-position switch라면 low는 172 근처, high는 1811 근처가 된다.
     */
    return raw > 1300;
}

static uint8_t CrsfCrc8DvbS2(const uint8_t* data, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; ++i)
    {
        crc ^= data[i];

        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x80)
            {
                crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
            }
            else
            {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }

    return crc;
}

static void UnpackCrsfChannels11bit(const uint8_t* payload,
                                    uint16_t ch[CRSF_NUM_CHANNELS])
{
    uint32_t bit_buffer = 0;
    int bit_count = 0;
    int byte_index = 0;

    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
    {
        while (bit_count < 11)
        {
            bit_buffer |= static_cast<uint32_t>(payload[byte_index++])
                          << bit_count;
            bit_count += 8;
        }

        ch[i] = static_cast<uint16_t>(bit_buffer & 0x07FF);
        bit_buffer >>= 11;
        bit_count -= 11;
    }
}


void KssCrsfReceiver::ResetParser()
{
    frame_index_ = 0;
    expected_total_len_ = 0;
    std::memset(frame_buf_, 0, sizeof(frame_buf_));
}

void KssCrsfReceiver::ProcessByte(uint8_t b)
{
    /*
     * CRSF frame:
     * [0] sync/address
     * [1] length
     * [2] type
     * [3 ...] payload
     * [last] crc
     *
     * length는 sync/address와 length byte를 제외한 길이.
     * 따라서 전체 frame 길이는 2 + length.
     */

    if (frame_index_ == 0)
    {
        if (b != CRSF_SYNC_BYTE)
        {
            return;
        }

        frame_buf_[frame_index_++] = b;
        return;
    }

    if (frame_index_ == 1)
    {
        const uint8_t len = b;

        if (len < CRSF_MIN_LENGTH || len > CRSF_MAX_LENGTH)
        {
            this->frame_error_count_.fetch_add(1, std::memory_order_relaxed);
            ResetParser();
            return;
        }

        frame_buf_[frame_index_++] = b;
        expected_total_len_ = 2 + len;

        if (expected_total_len_ > CRSF_MAX_FRAME_SIZE)
        {
            this->frame_error_count_.fetch_add(1, std::memory_order_relaxed);
            ResetParser();
            return;
        }

        return;
    }

    if (frame_index_ >= CRSF_MAX_FRAME_SIZE)
    {
        this->frame_error_count_.fetch_add(1, std::memory_order_relaxed);
        ResetParser();
        return;
    }

    frame_buf_[frame_index_++] = b;

    if (expected_total_len_ > 0 && frame_index_ >= expected_total_len_)
    {
        TryProcessFrame();
        ResetParser();
    }
}

void KssCrsfReceiver::TryProcessFrame()
{
    const uint8_t len = this->frame_buf_[1];

    if (len < CRSF_MIN_LENGTH || len > CRSF_MAX_LENGTH)
    {
        this->frame_error_count_.fetch_add(
            1,
            std::memory_order_relaxed
        );
        return;
    }

    const uint8_t type = this->frame_buf_[2];

    const uint8_t received_crc =
        this->frame_buf_[1 + len];

    const uint8_t calc_crc =
        CrsfCrc8DvbS2(&this->frame_buf_[2], len - 1);

    if (received_crc != calc_crc)
    {
        this->crc_error_count_.fetch_add(
            1,
            std::memory_order_relaxed
        );
        return;
    }

    this->rx_frame_count_.fetch_add(
        1,
        std::memory_order_relaxed
    );

    const uint8_t* payload = &this->frame_buf_[3];
    const int payload_len = len - 2;

    switch (type)
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        {
            if (this->DecodeRcChannels(payload, payload_len))
            {
                this->rc_frame_count_.fetch_add(
                    1,
                    std::memory_order_relaxed
                );
            }

            break;
        }

        case CRSF_FRAMETYPE_LINK_STATISTICS:
        {
            this->DecodeLinkStatistics(
                payload,
                payload_len
            );

            break;
        }

        default:
            break;
    }
}

bool KssCrsfReceiver::DecodeRcChannels(
    const uint8_t* payload,
    const int payload_len)
{
    if (payload == nullptr)
    {
        this->frame_error_count_.fetch_add(
            1,
            std::memory_order_relaxed
        );
        return false;
    }

    if (payload_len != CRSF_RC_CHANNEL_PAYLOAD_LEN)
    {
        this->frame_error_count_.fetch_add(
            1,
            std::memory_order_relaxed
        );
        return false;
    }

    uint16_t ch[CRSF_NUM_CHANNELS]{};
    UnpackCrsfChannels11bit(payload, ch);

    ControlPacket pkt{};
    if (!this->BuildControlPacketFromChannels(ch, pkt))
    {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();

    /*
     * SharedSnapshot::Write() 내부에서
     * SharedSnapshotFrame<ControlPacket>을 생성한다.
     */
    this->cmd_snapshot_.Write(pkt, now_us, 1);

    /*
     * 정상적인 RC channel frame이 처리된 시각.
     */
    this->last_rc_time_us_.store(
        now_us,
        std::memory_order_release
    );

    return true;
}

bool KssCrsfReceiver::GetLatestCommand(ControlPacket& out)
{
    SharedSnapshotFrame<ControlPacket> frame{};

    if (!this->cmd_snapshot_.Read(frame))
    {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();

    if ((now_us - frame.timestamp_us) > CRSF_COMMAND_TIMEOUT_US)
    {
        return false;
    }

    out = frame.data;
    return true;
}


bool KssCrsfReceiver::IsTimeout()
{
    SharedSnapshotFrame<ControlPacket> frame{};

    if (!this->cmd_snapshot_.Read(frame))
    {
        return true;
    }

    const int64_t now_us = esp_timer_get_time();

    return (now_us - frame.timestamp_us) > CRSF_COMMAND_TIMEOUT_US;
}

bool KssCrsfReceiver::BuildControlPacketFromChannels(
    const uint16_t ch[CRSF_NUM_CHANNELS],
    ControlPacket& out)
{
    out = ControlPacket{};

    out.type = PacketType::CONTROL;
    out.magic = 0xA5A5A5A5;

    /*
     * 모든 정상 RC channel frame마다 증가한다.
     */
    out.seq = ++this->seq_;

    const float roll =
        CrsfRawToNorm(ch[RC_CH_ROLL]);

    const float pitch =
        CrsfRawToNorm(ch[RC_CH_PITCH]);

    const float throttle =
        CrsfRawToThrottle01(ch[RC_CH_THROTTLE]);

    const float yaw =
        CrsfRawToNorm(ch[RC_CH_YAW]);

    const bool arm_switch =
        CrsfSwitchHigh(ch[RC_CH_ARM]);

    const bool mode_switch =
        CrsfSwitchHigh(ch[RC_CH_MODE]);

    const bool kill_switch =
        CrsfSwitchHigh(ch[RC_CH_KILL]);

    out.throttle = throttle;
    out.roll_rad = roll;
    out.pitch_rad = pitch;
    out.yaw_rate_rad_s = yaw;

    out.mode = mode_switch
        ? DroneMode::RATE_ACRO
        : DroneMode::ANGLE_SELF_LEVEL;

    /*
     * 첫 정상 RC frame에서는 현재 switch 상태만 획득한다.
     *
     * ARM switch가 HIGH인 상태로 FC/RX가 시작되어도
     * 자동 ARM_REQUEST를 발생시키지 않는다.
     */
    if (!this->switch_state_ready_)
    {
        this->prev_arm_switch_ = arm_switch;
        this->prev_mode_switch_ = mode_switch;
        this->prev_kill_switch_ = kill_switch;

        /*
        * ARM은 반드시 LOW를 한 번 확인한 뒤
        * LOW -> HIGH edge에서만 허용한다.
        */
        this->arm_low_seen_ = !arm_switch;
        this->switch_state_ready_ = true;

        /*
        * 시작 시 KILL이 이미 HIGH인 경우에도
        * emergency 명령을 발생시킨다.
        */
        if (kill_switch)
        {
            this->StartCommandEvent(
                static_cast<uint8_t>(CMD_EMERGENCY_STOP));
        }
        else
        {
        /*
         * 최초 CH6 상태를 FC mode에 동기화
         */
        this->StartCommandEvent(static_cast<uint8_t>(CMD_SET_MODE));
        }
    }
    else
    {
        /*
        * KILL rising edge: 최우선
        */
        if (kill_switch && !this->prev_kill_switch_)
        {
            this->StartCommandEvent(
                static_cast<uint8_t>(CMD_EMERGENCY_STOP));
        }
        /*
        * ARM falling edge: DISARM
        */
        else if (!arm_switch && this->prev_arm_switch_)
        {
            this->StartCommandEvent(
                static_cast<uint8_t>(CMD_DISARM_REQUEST));
        }
        /*
        * ARM rising edge:
        * 한 번 이상 LOW 확인 + KILL 해제 상태에서만 허용
        */
        else if (arm_switch &&
                !this->prev_arm_switch_ &&
                this->arm_low_seen_ &&
                !kill_switch)
        {
            this->StartCommandEvent(
                static_cast<uint8_t>(CMD_ARM_REQUEST));
        }
        /*
        * MODE 양방향 변경
        */
        else if (mode_switch != this->prev_mode_switch_)
        {
            this->StartCommandEvent(
                static_cast<uint8_t>(CMD_SET_MODE));
        }

        if (!arm_switch)
        {
            this->arm_low_seen_ = true;
        }

        /*
        * 다음 RC frame의 edge 검출 기준 갱신
        */
        this->prev_arm_switch_ = arm_switch;
        this->prev_mode_switch_ = mode_switch;
        this->prev_kill_switch_ = kill_switch;
    }

    /*
     * 새 명령이면 동일 cmd_seq/cmd_flags를 5회 반복하고,
     * 일반 frame이면 CMD_NONE을 넣는다.
     */
    this->FillCommandFields(out);

    return true;
}

//[Sync][Length][Type][Payload][CRC]

esp_err_t KssCrsfReceiver::SendFrame(
    const uint8_t frame_type,
    const uint8_t* payload,
    const size_t payload_len)
{
    if (!this->initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (payload == nullptr && payload_len > 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 전체 최대 64 bytes.
     *
     * Sync   : 1
     * Length : 1
     * Type   : 1
     * Payload
     * CRC    : 1
     *
     * 따라서 broadcast payload 최대값은 60 bytes.
     */
    if (payload_len > 60)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame[CRSF_MAX_FRAME_SIZE]{};

    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = static_cast<uint8_t>(payload_len + 2);
    frame[2] = frame_type;

    if (payload_len > 0)
    {
        std::memcpy(
            &frame[3],
            payload,
            payload_len
        );
    }

    /*
     * CRC 범위:
     * Type 1 byte + Payload
     */
    frame[3 + payload_len] =
        CrsfCrc8DvbS2(
            &frame[2],
            static_cast<int>(payload_len + 1)
        );

    const size_t total_len = payload_len + 4;

    const int written = uart_write_bytes(
        this->uart_num_,
        reinterpret_cast<const char*>(frame),
        total_len
    );

    if (written != static_cast<int>(total_len))
    {
        ESP_LOGE(
            TAG,
            "CRSF telemetry write failed: %d/%u",
            written,
            static_cast<unsigned>(total_len)
        );

        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t KssCrsfReceiver::SendFlightModeTelemetry(
    const uint8_t mode,
    const uint8_t state)
{
    char text[12]{};

    const DroneState drone_state =
        static_cast<DroneState>(state);

    const DroneMode drone_mode =
        static_cast<DroneMode>(mode);

    switch (drone_state)
    {
        case DroneState::INIT:
            std::strncpy(text, "INIT", sizeof(text) - 1);
            break;

        case DroneState::IMU_BIAS_CALIBRATING:
            std::strncpy(text, "CAL", sizeof(text) - 1);
            break;

        case DroneState::LANDING:
            std::strncpy(text, "LAND", sizeof(text) - 1);
            break;

        case DroneState::ERR:
            std::strncpy(text, "!ERR!", sizeof(text) - 1);
            break;

        case DroneState::DISARMED:
            if (drone_mode == DroneMode::ANGLE_SELF_LEVEL)
            {
                std::strncpy(text, "ANGL*", sizeof(text) - 1);
            }
            else
            {
                std::strncpy(text, "ACRO*", sizeof(text) - 1);
            }
            break;

        case DroneState::ARMING:
            std::strncpy(text, "ARMING", sizeof(text) - 1);
            break;

        case DroneState::ARMED:
            if (drone_mode == DroneMode::ANGLE_SELF_LEVEL)
            {
                std::strncpy(text, "ANGL", sizeof(text) - 1);
            }
            else
            {
                std::strncpy(text, "ACRO", sizeof(text) - 1);
            }
            break;

        default:
            std::strncpy(text, "UNKNOWN", sizeof(text) - 1);
            break;
    }

    const size_t payload_len =
        std::strlen(text) + 1; // null 포함

    return this->SendFrame(
        CRSF_FRAMETYPE_FLIGHT_MODE,
        reinterpret_cast<const uint8_t*>(text),
        payload_len
    );
}

esp_err_t KssCrsfReceiver::SendTelemetry(
    const TelemetryPacket& telemetry)
{
    esp_err_t ret = ESP_OK;

    switch (this->telemetry_slot_)
    {
        case 0:
            ret = this->SendBatteryTelemetry(
                telemetry.battery_voltage,
                0.0f,
                0,
                telemetry.battery_percent
            );
            break;

        case 1:
            ret = this->SendFlightModeTelemetry(
                telemetry.mode,
                telemetry.state
            );
            break;

        default:
            break;
    }

    this->telemetry_slot_ =
        static_cast<uint8_t>(
            (this->telemetry_slot_ + 1) % 2
        );

    return ret;
}

void KssCrsfReceiver::StartCommandEvent(
    const uint8_t cmd_flags)
{
    if (cmd_flags == static_cast<uint8_t>(CMD_NONE))
    {
        return;
    }

    /*
     * 새 명령 이벤트에서만 증가한다.
     * uint8_t wrap-around는 허용한다.
     */
    ++this->cmd_seq_;

    this->pending_cmd_flags_ = cmd_flags;
    this->command_repeat_remaining_ =
        COMMAND_REPEAT_COUNT;
}

void KssCrsfReceiver::FillCommandFields(
    ControlPacket& out)
{
    /*
     * cmd_seq는 새 이벤트가 발생할 때만 바뀐다.
     */
    out.cmd_seq = this->cmd_seq_;

    if (this->command_repeat_remaining_ == 0)
    {
        out.cmd_flags =
            static_cast<uint8_t>(CMD_NONE);
        return;
    }

    /*
     * 같은 cmd_seq와 cmd_flags를 여러 RC 프레임에 반복한다.
     */
    out.cmd_flags = this->pending_cmd_flags_;

    --this->command_repeat_remaining_;

    if (this->command_repeat_remaining_ == 0)
    {
        this->pending_cmd_flags_ =
            static_cast<uint8_t>(CMD_NONE);
    }
}

bool KssCrsfReceiver::DecodeLinkStatistics(
    const uint8_t* payload,
    const int payload_len)
{
    if (payload == nullptr || payload_len < 10)
    {
        this->frame_error_count_.fetch_add(
            1,
            std::memory_order_relaxed
        );
        return false;
    }

    CrsfLinkStatistics stats{};

    stats.uplink_rssi_ant1_dbm =
        -static_cast<int16_t>(payload[0]);

    stats.uplink_rssi_ant2_dbm =
        -static_cast<int16_t>(payload[1]);

    stats.uplink_link_quality = payload[2];

    stats.uplink_snr_db =
        static_cast<int8_t>(payload[3]);

    stats.active_antenna = payload[4];
    stats.rf_profile = payload[5];
    stats.uplink_rf_power_code = payload[6];

    stats.downlink_rssi_dbm =
        -static_cast<int16_t>(payload[7]);

    stats.downlink_link_quality = payload[8];

    stats.downlink_snr_db =
        static_cast<int8_t>(payload[9]);

    stats.timestamp_us = esp_timer_get_time();

    this->link_stats_snapshot_.Write(
        stats,
        stats.timestamp_us,
        1
    );

    return true;
}

esp_err_t KssCrsfReceiver::SendBatteryTelemetry(
    const float voltage_v,
    const float current_a,
    const uint32_t consumed_mah,
    const uint8_t remaining_percent)
{
    uint8_t payload[8]{};

    const uint16_t voltage_raw =
        static_cast<uint16_t>(
            std::lround(voltage_v * 10.0f)
        );

    const uint16_t current_raw =
        static_cast<uint16_t>(
            std::lround(
                std::max(0.0f, current_a) * 10.0f
            )
        );

    WriteU16Be(&payload[0], voltage_raw);
    WriteU16Be(&payload[2], current_raw);

    const uint32_t capacity =
        std::min(consumed_mah, 0xFFFFFFUL);

    payload[4] =
        static_cast<uint8_t>(capacity >> 16);

    payload[5] =
        static_cast<uint8_t>(capacity >> 8);

    payload[6] =
        static_cast<uint8_t>(capacity);

    payload[7] =
        std::min<uint8_t>(remaining_percent, 100);

    return this->SendFrame(
        CRSF_FRAMETYPE_BATTERY_SENSOR,
        payload,
        sizeof(payload)
    );
}

bool KssCrsfReceiver::GetLinkStatistics(CrsfLinkStatistics& out)
{
    SharedSnapshotFrame<CrsfLinkStatistics> frame{};

    if (!this->link_stats_snapshot_.Read(frame))
    {
        return false;
    }

    out = frame.data;
    return true;
}