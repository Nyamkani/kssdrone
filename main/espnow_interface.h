#pragma once

#include <cstdint>

/* ESP-IDF */
#include "esp_err.h"
#include "esp_now.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "packets.h"
#include "shared_snapshot.h"

enum class ESPNOWState
{
    INIT = 0,
    RUN  = 1,
    STOP = 2,
    ERR  = 10,
};

struct EspNowRxStats
{
    uint32_t raw_count = 0;
    uint32_t valid_count = 0;
    uint32_t magic_ok_count = 0;
    uint32_t queue_ok_count = 0;

    uint32_t rx_count = 0;
    uint32_t seq_jump_count = 0;
    uint32_t last_seq = 0;

    int64_t last_rx_time_us = 0;
    int64_t max_rx_gap_us = 0;
};

class EspNowInterface
{
public:
    EspNowInterface() = default;
    ~EspNowInterface() = default;

    esp_err_t Initialize();
    esp_err_t StartTask();
    void StopTaskRequest();

    esp_err_t Init();
    esp_err_t Run();
    esp_err_t Error();

    bool GetLatestCommand(ControlPacket& out);
    bool IsTimeout();

    esp_err_t SendTelemetry(const TelemetryPacket& td);

    uint32_t GetRawCount();
    uint32_t GetValidCount();
    uint32_t GetMagicOkCount();
    uint32_t GetQueueOkCount();

    uint32_t GetRxCount();
    uint32_t GetSeqJumpCount();
    uint32_t GetLastSeq();

    int64_t GetLastRxTimeUs();
    int64_t GetMaxRxGapUs();
    void ResetMaxRxGap();

private:
    static void MainTask(void *param);

    static void EspNowRecvCb(const esp_now_recv_info_t *info,
                             const uint8_t *data,
                             int len);

    bool IsMacValid(const uint8_t mac[6]) const;
    void MainLoop();

    bool IsTaskStopRequested();
    void SetTaskStop(bool stop);

    EspNowRxStats GetStatsSnapshot();

private:
    ESPNOWState state_ = ESPNOWState::INIT;
    bool hw_initialized_ = false;

    /*
     * task stop flag 보호용
     */
    portMUX_TYPE task_mux_ = portMUX_INITIALIZER_UNLOCKED;
    bool task_stop_ = false;

    /*
     * RX queue: callback -> espnow task
     */
    QueueHandle_t rx_queue_ = nullptr;
    TaskHandle_t handle_ = nullptr;

    /*
     * Latest command snapshot:
     * espnow task에서 Write
     * main task에서 Read
     */
    TripleSnapshot<ControlPacket> cmd_snapshot_;

    /*
     * RX stats
     */
    portMUX_TYPE stats_mux_ = portMUX_INITIALIZER_UNLOCKED;
    EspNowRxStats stats_{};

    static constexpr int RX_QUEUE_SIZE = 1;
    static constexpr int64_t TIMEOUT_US = 300000; // 300ms

    static constexpr uint8_t CONTROLLER_MAC[6] = {
        0x1C, 0xDB, 0xD4, 0x76, 0x1B, 0x84
    };

    static constexpr uint8_t ESPNOW_CHANNEL = 1;

    static EspNowInterface* instance_;

    uint32_t telemetry_seq_ = 0;
};