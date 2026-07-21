#pragma once

#include <cstdint>
#include <atomic>

/* ESP-IDF */
#include "esp_err.h"
#include "esp_now.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "packets.h"


enum class ESPNOWState
{
    INIT = 0,
    RUN = 1, 
    STOP = 2,   
    ERR = 10,

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

    bool GetLatestCommand(ControlPacket& out) const;
    bool IsTimeout() const;

    esp_err_t SendTelemetry(const TelemetryPacket& td);


    uint32_t GetRawCount() const;
    uint32_t GetValidCount() const;
    uint32_t GetMagicOkCount() const;
    uint32_t GetQueueOkCount() const;

    uint32_t GetRxCount() const;
    uint32_t GetSeqJumpCount() const;
    uint32_t GetLastSeq() const;

    int64_t GetLastRxTimeUs() const;
    int64_t GetMaxRxGapUs() const;
    void ResetMaxRxGap();

private:
    static void MainTask(void *param);
    static void EspNowRecvCb(const esp_now_recv_info_t *info,
                             const uint8_t *data,
                             int len);
    bool IsMacValid(const uint8_t mac[6]) const;
    void MainLoop();

private:
    ESPNOWState state_ = ESPNOWState::INIT;
    std::atomic<bool> task_stop_{false};
    bool hw_initialized_ = false;

    /* RX queue: callback -> task */
    QueueHandle_t rx_queue_ = nullptr;
    TaskHandle_t handle_ = nullptr;

    /* Double buffer */
    ControlPacket cmd_buf_[2]{};

    // read_idx_ points to the latest completed packet.
    // write_idx_ is the buffer index used by producer task.
    std::atomic<int> read_idx_{0};
    std::atomic<int> write_idx_{1};
 

    /* Config */
    static constexpr int RX_QUEUE_SIZE = 1;   //due to overwrite behavior, we only need a queue of size 1 to store the latest command packet. If the queue is full, we can just drop the old packet and replace it with the new one.
    static constexpr int64_t TIMEOUT_US = 300000; // 300ms
    
    /* MAC placeholder */
    static constexpr uint8_t CONTROLLER_MAC[6] = {
        0x1C, 0xDB, 0xD4, 0x76, 0x1B, 0x84
    };

    static constexpr uint8_t  ESPNOW_CHANNEL = 1;

    static EspNowInterface* instance_;

    /* Timeout */
    std::atomic<uint32_t> rx_raw_count_{0};
    std::atomic<uint32_t> rx_valid_count_{0};
    std::atomic<uint32_t> rx_magic_ok_count_{0};
    std::atomic<uint32_t> rx_queue_ok_count_{0};

    std::atomic<uint32_t> rx_count_{0};
    std::atomic<uint32_t> seq_jump_count_{0};
    std::atomic<uint32_t> last_seq_{0};

    std::atomic<int64_t> last_rx_time_us_{0};
    std::atomic<int64_t> max_rx_gap_us_{0};

};