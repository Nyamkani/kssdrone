#include "espnow_interface.h"

#include <cstring>

/* ESP-IDF */
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

EspNowInterface* EspNowInterface::instance_ = nullptr;
static const char* TAG = "ESPNOW_IFACE";

// void EspNowInterface::EspNowRecvCb(const esp_now_recv_info_t *info,
//                                    const uint8_t *data,
//                                    int len)
// {

//     // ESP_LOGI(TAG,
//     //     "RX raw len=%d time=%lld",
//     //     len,
//     //     esp_timer_get_time()
//     // );

//     if (instance_ == nullptr || data == nullptr || len < sizeof(PacketType))
//         return;

//     PacketType type;
//     std::memcpy(&type, data, sizeof(PacketType));

//     switch(type)
//     {
//         case PacketType::CONTROL:
//         {
//             if(len != sizeof(ControlPacket))
//                 return; // Invalid packet length for control packet, ignore it.

//             ControlPacket pkt{};
//             std::memcpy(&pkt, data, sizeof(ControlPacket));
            
//             if (pkt.magic != 0xA5A5A5A5)
//                 return;

//             instance_->last_rx_time_us_.store(esp_timer_get_time());

//             // Handle control packet
//             if (instance_->rx_queue_ != nullptr)
//             {
//                 // callback has to be fast, so we just push the raw packet to the queue and let the task handle it.
//                 // xQueueSend(instance_->rx_queue_, &pkt, 0);
//                    BaseType_t hp = pdFALSE;

//                 xQueueOverwriteFromISR(instance_->rx_queue_, &pkt, &hp);

//                 if (hp)
//                     portYIELD_FROM_ISR();
//             }

//             break;
//         }
//         case PacketType::TELEMETRY:
//         {
//             // Handle telemetry packet
//             break;

//         }

//         default:
//             return; // Unknown packet type, ignore it.
//     }
// }

void EspNowInterface::EspNowRecvCb(const esp_now_recv_info_t *info,
                                   const uint8_t *data,
                                   int len)
{
    if (instance_ == nullptr || data == nullptr)
        return;

    instance_->rx_raw_count_.fetch_add(1);

    if (len < sizeof(PacketType))
        return;

    PacketType type{};
    std::memcpy(&type, data, sizeof(PacketType));

    if (type != PacketType::CONTROL)
        return;

    if (len != sizeof(ControlPacket))
        return;

    instance_->rx_valid_count_.fetch_add(1);

    ControlPacket pkt{};
    std::memcpy(&pkt, data, sizeof(ControlPacket));

    if (pkt.magic != 0xA5A5A5A5)
        return;

    instance_->rx_magic_ok_count_.fetch_add(1);

    const int64_t now = esp_timer_get_time();

    const int64_t prev_time = instance_->last_rx_time_us_.load();
    const uint32_t prev_seq = instance_->last_seq_.load();

    if (prev_time > 0)
    {
        const int64_t gap = now - prev_time;

        int64_t old_max = instance_->max_rx_gap_us_.load();
        while (gap > old_max &&
               !instance_->max_rx_gap_us_.compare_exchange_weak(old_max, gap))
        {
        }

        if (pkt.seq != prev_seq + 1)
        {
            instance_->seq_jump_count_.fetch_add(1);
        }
    }

    instance_->last_seq_.store(pkt.seq);
    instance_->last_rx_time_us_.store(now);
    instance_->rx_count_.fetch_add(1);

    if (instance_->rx_queue_ != nullptr)
    {
        BaseType_t hp = pdFALSE;

        BaseType_t ok =
            xQueueOverwriteFromISR(instance_->rx_queue_, &pkt, &hp);

        if (ok == pdTRUE)
        {
            instance_->rx_queue_ok_count_.fetch_add(1);
        }

        if (hp)
            portYIELD_FROM_ISR();
    }
}

esp_err_t EspNowInterface::SendTelemetry(const TelemetryPacket& td)
{
    esp_err_t ret = ESP_OK;

    TelemetryPacket td_copy = td; // Make a copy to modify the type field if necessary

    // Prepare telemetry packet
    td_copy.type = PacketType::TELEMETRY; // Ensure the packet type is set correctly

    td_copy.seq++;
    // Send the telemetry packet
    ret = esp_now_send(CONTROLLER_MAC, reinterpret_cast<const uint8_t*>(&td_copy), sizeof(td_copy));

    return ret;
}



//////////////////////////////////////////////////////////////////////////


bool EspNowInterface::IsMacValid(const uint8_t mac[6]) const
{
    for (int i = 0; i < 6; ++i)
    {
        if (mac[i] != 0x00)
            return true;
    }
    return false;
}

bool EspNowInterface::GetLatestCommand(ControlPacket& out) const
{
    if (this->IsTimeout())
    {
        return false;
    }

    // Snapshot current published buffer index
    const int idx = this->read_idx_.load();
    out = cmd_buf_[idx];
    return true;
}

bool EspNowInterface::IsTimeout() const
{
    const int64_t now = esp_timer_get_time();
    const int64_t last_rx = this->last_rx_time_us_.load();
    const int64_t dt = now - last_rx;
    return (dt) > TIMEOUT_US;
}   

//////////////////////////////////////////////////////////////////////////


void EspNowInterface::MainTask(void *param)
{
    auto* this_ = static_cast<EspNowInterface*>(param);

    if (this_ == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }   

    this_->MainLoop();

    this_->handle_ = nullptr;

    vTaskDelete(nullptr);

    return;
}

void EspNowInterface::MainLoop()
{
   while (true)
    {
        if (this->task_stop_.load())
            break;

        switch(this->state_)
        {
            case ESPNOWState::INIT:
                /* Perform initialization tasks, e.g., check IMU status, configure settings */
                // If initialization successful:
                if(this->Init() == ESP_OK)
                {
                    this->state_ = ESPNOWState::RUN;
                }
                // Else if initialization failed:
                else
                {
                    this->state_ = ESPNOWState::ERR;
                }
                break;

            case ESPNOWState::RUN:
                if(this->Run() != ESP_OK)
                {
                    this->state_ = ESPNOWState::ERR;
                }

                break;

            case ESPNOWState::STOP:
                /* Handle stop state, e.g., pause data acquisition */
                break;

            case ESPNOWState::ERR:
                /* Handle error state, e.g., attempt recovery or log error */
                this->Error();
                break;

            default:
                /* Handle unexpected state */
                break;
        }
    }
    return;
}



esp_err_t EspNowInterface::Init()
{
    this->read_idx_.store(0);
    this->write_idx_.store(1);

    this->last_rx_time_us_.store(esp_timer_get_time());

    return ESP_OK;
}


esp_err_t EspNowInterface::Run()
{
    if (!this->hw_initialized_)
    {
        return ESP_OK;
    }

    ControlPacket pkt{};

    if (xQueueReceive(this->rx_queue_, &pkt, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        if (pkt.magic != 0xA5A5A5A5)
            return ESP_OK;

        const int w = this->write_idx_.load();

        this->cmd_buf_[w] = pkt;

        this->read_idx_.store(w);
        this->write_idx_.store(1 - w);
    }

    return ESP_OK;
}


esp_err_t EspNowInterface::Error()
{
    vTaskDelay(pdMS_TO_TICKS(100));
    this->state_ = ESPNOWState::INIT;
    return ESP_OK;
}


esp_err_t EspNowInterface::StartTask()
{
    esp_err_t ret = xTaskCreatePinnedToCore(
        MainTask,
        "espnow_main_task",
        4096*2,
        this,
        5,
        &this->handle_,
        0
    );

    if (ret == 1)
    {
       ret = ESP_OK;
    }
    else 
    {
        ret = ESP_FAIL;
    }

    return ret;
}

void EspNowInterface::StopTaskRequest()
{
    this->task_stop_.store(true);

    TaskHandle_t handle = this->handle_;

    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}


///////////////////////////////////////////////////////////////

// EspNowInterface::EspNowInterface()
// {

// }

// EspNowInterface::~EspNowInterface()
// {

// }

esp_err_t EspNowInterface::Initialize()
{
    instance_ = this;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK)
        return ret;

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        return ret;

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(100)); 


   esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (sta_netif == NULL) {
        // ESP_LOGI("INIT", "Creating new default STA netif");
        sta_netif = esp_netif_create_default_wifi_sta();
    } else {
        // ESP_LOGI("INIT", "Using existing STA netif");
    }

    // esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == nullptr)
        return ESP_FAIL;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //STAMODE
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    //APMODE
    /*   
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

    wifi_config_t ap_cfg = {};
    std::strncpy(reinterpret_cast<char*>(ap_cfg.ap.ssid),
                 "KSS-DRONE",
                 sizeof(ap_cfg.ap.ssid));
    std::strncpy(reinterpret_cast<char*>(ap_cfg.ap.password),
                 "12345678",
                 sizeof(ap_cfg.ap.password));

    ap_cfg.ap.channel = 1;
    ap_cfg.ap.max_connection = 1;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    */
    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    rx_queue_ = xQueueCreate(RX_QUEUE_SIZE, sizeof(ControlPacket));
    if (rx_queue_ == nullptr)
        return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(EspNowRecvCb));

    if (IsMacValid(CONTROLLER_MAC))
    {
        esp_now_peer_info_t peer = {};
        std::memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;

        if (!esp_now_is_peer_exist(CONTROLLER_MAC))
        {
            // ESP_ERROR_CHECK(esp_now_add_peer(&peer));

            esp_err_t add_ret = esp_now_add_peer(&peer);
            if (add_ret != ESP_OK)
            {
                return add_ret;
            }
        }
    }
    else    
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->hw_initialized_ = true;

    return this->StartTask();
}

uint32_t EspNowInterface::GetRawCount() const
{
    return this->rx_raw_count_.load();
}

uint32_t EspNowInterface::GetValidCount() const
{
    return this->rx_valid_count_.load();
}

uint32_t EspNowInterface::GetMagicOkCount() const
{
    return this->rx_magic_ok_count_.load();
}

uint32_t EspNowInterface::GetQueueOkCount() const
{
    return this->rx_queue_ok_count_.load();
}

uint32_t EspNowInterface::GetRxCount() const
{
    return this->rx_count_.load();
}

uint32_t EspNowInterface::GetSeqJumpCount() const
{
    return this->seq_jump_count_.load();
}

uint32_t EspNowInterface::GetLastSeq() const
{
    return this->last_seq_.load();
}

int64_t EspNowInterface::GetLastRxTimeUs() const
{
    return this->last_rx_time_us_.load();
}

int64_t EspNowInterface::GetMaxRxGapUs() const
{
    return this->max_rx_gap_us_.load();
}

void EspNowInterface::ResetMaxRxGap()
{
    this->max_rx_gap_us_.store(0);
}