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

////////////////////////////////////////////////////////////////////////

void EspNowInterface::EspNowRecvCb(const esp_now_recv_info_t *info,
                                   const uint8_t *data,
                                   int len)
{
    (void)info;

    EspNowInterface* self = instance_;
    if (self == nullptr || data == nullptr)
    {
        return;
    }

    /*
     * callback 수신 count만 기록.
     * 파싱은 espnow task에서 한다.
     */
    portENTER_CRITICAL(&self->stats_mux_);
    self->stats_.raw_count++;
    portEXIT_CRITICAL(&self->stats_mux_);

    if (len != static_cast<int>(sizeof(ControlPacket)))
    {
        return;
    }

    ControlPacket pkt{};
    std::memcpy(&pkt, data, sizeof(ControlPacket));

    if (self->rx_queue_ != nullptr)
    {
        BaseType_t hp = pdFALSE;

        BaseType_t ok =
            xQueueOverwriteFromISR(self->rx_queue_, &pkt, &hp);

        if (ok == pdTRUE)
        {
            portENTER_CRITICAL(&self->stats_mux_);
            self->stats_.queue_ok_count++;
            portEXIT_CRITICAL(&self->stats_mux_);
        }

        if (hp)
        {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t EspNowInterface::SendTelemetry(const TelemetryPacket& td)
{
    if (!this->hw_initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    TelemetryPacket td_copy = td;

    td_copy.type = PacketType::TELEMETRY;
    td_copy.seq = ++this->telemetry_seq_;

    return esp_now_send(
        CONTROLLER_MAC,
        reinterpret_cast<const uint8_t*>(&td_copy),
        sizeof(td_copy)
    );
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

EspNowRxStats EspNowInterface::GetStatsSnapshot()
{
    EspNowRxStats copy{};

    portENTER_CRITICAL(&this->stats_mux_);

    copy = this->stats_;

    portEXIT_CRITICAL(&this->stats_mux_);

    return copy;
}

bool EspNowInterface::GetLatestCommand(ControlPacket& out)
{
    SharedSnapshotFrame<ControlPacket> frame{};

    if (!this->cmd_snapshot_.Read(frame))
    {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();

    if ((now_us - frame.timestamp_us) > TIMEOUT_US)
    {
        return false;
    }

    out = frame.data;
    return true;
}

bool EspNowInterface::IsTimeout()
{
    SharedSnapshotFrame<ControlPacket> frame{};

    if (!this->cmd_snapshot_.Read(frame))
    {
        return true;
    }

    const int64_t now_us = esp_timer_get_time();

    return (now_us - frame.timestamp_us) > TIMEOUT_US;
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
        if (this->IsTaskStopRequested())
        {
            break;
        }

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
    portENTER_CRITICAL(&this->stats_mux_);

    this->stats_ = {};

    portEXIT_CRITICAL(&this->stats_mux_);

    return ESP_OK;
}

esp_err_t EspNowInterface::Run()
{
    if (!this->hw_initialized_)
    {
        return ESP_OK;
    }

    ControlPacket pkt{};

    if (xQueueReceive(this->rx_queue_, &pkt, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        return ESP_OK;
    }

    const int64_t now_us = esp_timer_get_time();

    bool publish = false;

    portENTER_CRITICAL(&this->stats_mux_);

    if (pkt.type == PacketType::CONTROL)
    {
        this->stats_.valid_count++;

        if (pkt.magic == 0xA5A5A5A5)
        {
            this->stats_.magic_ok_count++;

            if (this->stats_.last_rx_time_us > 0)
            {
                const int64_t gap =
                    now_us - this->stats_.last_rx_time_us;

                if (gap > this->stats_.max_rx_gap_us)
                {
                    this->stats_.max_rx_gap_us = gap;
                }

                if (pkt.seq != this->stats_.last_seq + 1)
                {
                    this->stats_.seq_jump_count++;
                }
            }

            this->stats_.last_seq = pkt.seq;
            this->stats_.last_rx_time_us = now_us;

            this->stats_.rx_count++;

            publish = true;
        }
    }

    portEXIT_CRITICAL(&this->stats_mux_);

    if (publish)
    {
        this->cmd_snapshot_.Write(pkt, now_us, 1);
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
    this->SetTaskStop(false);

    BaseType_t ret = xTaskCreatePinnedToCore(
        MainTask,
        "espnow_main_task",
        4096 * 2,
        this,
        5,
        &this->handle_,
        0
    );

    if (ret == pdPASS)
    {
        return ESP_OK;
    }

    this->handle_ = nullptr;
    return ESP_FAIL;
}


void EspNowInterface::StopTaskRequest()
{
    this->SetTaskStop(true);

    TaskHandle_t handle = this->handle_;

    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}

bool EspNowInterface::IsTaskStopRequested()
{
    bool stop = false;

    portENTER_CRITICAL(&this->task_mux_);

    stop = this->task_stop_;

    portEXIT_CRITICAL(&this->task_mux_);

    return stop;
}

void EspNowInterface::SetTaskStop(bool stop)
{
    portENTER_CRITICAL(&this->task_mux_);

    this->task_stop_ = stop;

    portEXIT_CRITICAL(&this->task_mux_);
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

uint32_t EspNowInterface::GetRawCount()
{
    return this->GetStatsSnapshot().raw_count;
}

uint32_t EspNowInterface::GetValidCount()
{
    return this->GetStatsSnapshot().valid_count;
}

uint32_t EspNowInterface::GetMagicOkCount()
{
    return this->GetStatsSnapshot().magic_ok_count;
}

uint32_t EspNowInterface::GetQueueOkCount()
{
    return this->GetStatsSnapshot().queue_ok_count;
}

uint32_t EspNowInterface::GetRxCount()
{
    return this->GetStatsSnapshot().rx_count;
}

uint32_t EspNowInterface::GetSeqJumpCount()
{
    return this->GetStatsSnapshot().seq_jump_count;
}

uint32_t EspNowInterface::GetLastSeq()
{
    return this->GetStatsSnapshot().last_seq;
}

int64_t EspNowInterface::GetLastRxTimeUs()
{
    return this->GetStatsSnapshot().last_rx_time_us;
}

int64_t EspNowInterface::GetMaxRxGapUs()
{
    return this->GetStatsSnapshot().max_rx_gap_us;
}

void EspNowInterface::ResetMaxRxGap()
{
    portENTER_CRITICAL(&this->stats_mux_);

    this->stats_.max_rx_gap_us = 0;

    portEXIT_CRITICAL(&this->stats_mux_);
}