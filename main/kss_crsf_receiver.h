#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <atomic>
#include <cmath>
#include "driver/uart.h"

#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"


#include "packets.h"   
#include "shared_snapshot.h"

/*
*바이트 위치명칭실제 값 (Hex)의미
*1번째 Sync Byte 0xC8 (또는 0xC4)"지금부터 데이터 시작한다"는 신호
*2번째 Length 0x18 (십진수 24)뒤에 이어지는 데이터가 총 24바이트라는 뜻
*3번째 Type 0x16 "이 패킷은 조종 스틱(채널) 데이터다"
*4~25번째 Payload 데이터 22바이트 실제 16개 채널의 조종 값 (압축됨)
*26번째 CRC 1바이트 데이터 오류 검사용 체크섬
*/

/*
 * CRSF 기본 상수
 */

//Sender command type
static constexpr uint8_t CRSF_FRAMETYPE_FLIGHT_MODE = 0x21;
static constexpr uint8_t CRSF_FRAMETYPE_GPS = 0x02;
static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
static constexpr uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08;

//Receiver command type
static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;


static constexpr int CRSF_MAX_FRAME_SIZE = 64;
static constexpr int CRSF_MIN_LENGTH = 2;
static constexpr int CRSF_MAX_LENGTH = 62;

static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

static constexpr int CRSF_RC_CHANNEL_PAYLOAD_LEN = 22;
static constexpr int CRSF_NUM_CHANNELS = 16;


static constexpr int64_t CRSF_COMMAND_TIMEOUT_US = 500*1000;
/*
 * Channel mapping.
 *
 * 주의:
 * Mode 2는 조종기 스틱의 물리 배치이고,
 * 여기의 CH 번호는 CRSF로 들어오는 논리 채널 번호이다.
 *
 * 모듈 도착 후 로그로 실제 채널을 확인하고 이 값만 바꾸면 된다.
 */
static constexpr int RC_CH_ROLL     = 0; // CH1
static constexpr int RC_CH_PITCH    = 1; // CH2
static constexpr int RC_CH_THROTTLE = 2; // CH3
static constexpr int RC_CH_YAW      = 3; // CH4

static constexpr int RC_CH_ARM      = 4; // CH5
static constexpr int RC_CH_MODE     = 5; // CH6
static constexpr int RC_CH_KILL     = 6; // CH7

/*
 * CRSF raw range.
 * 일반적으로 center는 약 992, low/high는 약 172/1811 근처.
 * 실제 RadioMaster T8L 연결 후 로그로 확인하고 필요 시 조정.
 */
static constexpr float CRSF_RAW_MIN = 172.0f;
static constexpr float CRSF_RAW_MID = 992.0f;
static constexpr float CRSF_RAW_MAX = 1811.0f;


struct CrsfLinkStatistics
{
    int16_t uplink_rssi_ant1_dbm{0};
    int16_t uplink_rssi_ant2_dbm{0};

    uint8_t uplink_link_quality{0};
    int8_t uplink_snr_db{0};

    uint8_t active_antenna{0};
    uint8_t rf_profile{0};
    uint8_t uplink_rf_power_code{0};

    int16_t downlink_rssi_dbm{0};
    uint8_t downlink_link_quality{0};
    int8_t downlink_snr_db{0};

    int64_t timestamp_us{0};
};



class KssCrsfReceiver
{
    public:
        KssCrsfReceiver() = default;

        esp_err_t Initialize(uart_port_t uart_num, QueueHandle_t uart_event_queue);
        esp_err_t StartTask();

        void StopTaskRequest();
        void MainLoop();

        int64_t GetLastRcTimeUs() const
        {
            return this->last_rc_time_us_.load(std::memory_order_relaxed);
        }

        uint32_t GetRxFrameCount() const
        {
            return this->rx_frame_count_.load(std::memory_order_relaxed);
        }

        uint32_t GetRcFrameCount() const
        {
            return this->rc_frame_count_.load(std::memory_order_relaxed);
        }

        uint32_t GetCrcErrorCount() const
        {
            return this->crc_error_count_.load(std::memory_order_relaxed);
        }

        uint32_t GetFrameErrorCount() const
        {
            return this->frame_error_count_.load(std::memory_order_relaxed);
        }

        bool GetLatestCommand(ControlPacket& out);

        esp_err_t SendTelemetry(const TelemetryPacket& telemetry);

        bool IsTimeout();

        bool DecodeLinkStatistics(const uint8_t* payload, int payload_len);

        bool GetLinkStatistics(
            CrsfLinkStatistics& out
        );


    private:
        static void MainTask(void* arg);

        void ProcessByte(uint8_t b);
        void ResetParser();
        void TryProcessFrame();

        bool DecodeRcChannels(const uint8_t* payload, int payload_len);
        bool BuildControlPacketFromChannels(const uint16_t ch[CRSF_NUM_CHANNELS],
                                            ControlPacket& out);

        esp_err_t SendFrame(uint8_t frame_type, const uint8_t* payload,size_t payload_len);

        esp_err_t SendFlightModeTelemetry(uint8_t mode,uint8_t state);

        esp_err_t SendBatteryTelemetry(
            const float voltage_v,
            const float current_a,
            const uint32_t consumed_mah,
            const uint8_t remaining_percent);

        void StartCommandEvent(uint8_t cmd_flags);
        void FillCommandFields(ControlPacket& out);

    private:
        uart_port_t uart_num_{UART_NUM_1};
        QueueHandle_t uart_event_queue_{nullptr};
        TaskHandle_t task_handle_{nullptr};
        TripleSnapshot<ControlPacket> cmd_snapshot_;

        TripleSnapshot<CrsfLinkStatistics>link_stats_snapshot_;

        std::atomic_bool stop_requested_{false};
        bool initialized_{false};

        uint8_t frame_buf_[64]{};
        int frame_index_{0};
        int expected_total_len_{0};

        static constexpr uint8_t COMMAND_REPEAT_COUNT = 5;

        uint16_t seq_{0};
        uint8_t cmd_seq_{0};

        uint8_t pending_cmd_flags_{static_cast<uint8_t>(CMD_NONE)};

        uint8_t command_repeat_remaining_{0};

        bool switch_state_ready_{false};
        bool arm_low_seen_{false};

        bool prev_arm_switch_{false};
        bool prev_mode_switch_{false};
        bool prev_kill_switch_{false};

        std::atomic_int64_t last_rc_time_us_{0};

        std::atomic_uint32_t rx_frame_count_{0};
        std::atomic_uint32_t rc_frame_count_{0};
        std::atomic_uint32_t crc_error_count_{0};
        std::atomic_uint32_t frame_error_count_{0};

        uint8_t telemetry_slot_{0};
};


inline static void WriteU16Be(
    uint8_t* dst,
    const uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value >> 8);
    dst[1] = static_cast<uint8_t>(value);
}