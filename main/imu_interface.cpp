#include "imu_interface.h"
#include "pins.h"
#include "driver/gpio.h"

static void IRAM_ATTR imu_int_isr(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;

    IMUInterface* this_ = static_cast<IMUInterface*>(arg);
    
    if (this_ != nullptr && this_->handle_ != nullptr)
    {
        vTaskNotifyGiveFromISR(this_->handle_, &high_task_wakeup);
    }

    if (high_task_wakeup)
    {
        portYIELD_FROM_ISR();
    }
}

void IMUInterface::IMURegisterInterrupt()
{
    // gpio_isr_handler_add(PIN_IMU_INT, imu_int_isr, this);
    BoardInstallIMUISR(imu_int_isr, this);
    
    return;
}

void IMUInterface::IMUUnregisterInterrupt()
{
    gpio_isr_handler_remove(PIN_IMU_INT);
}


////////////////////////////////////////////////////////////////////////////////


void IMUInterface::StartTask() 
{
    if(this->handle_ == nullptr) 
    {   
        this->task_stop_.store(false);

        BaseType_t ret = xTaskCreatePinnedToCore(MainTask, "IMUTask", 4096, this, 7, &(this->handle_), 1);

        if (ret != pdPASS)
        {
            this->handle_ = nullptr;
        }
    }

    return;
}

void IMUInterface::StopTaskRequest()
{
    this->task_stop_.store(true);

    TaskHandle_t handle = this->handle_;

    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}

void IMUInterface::MainTask(void* param) 
{
    /*Initialization*/
    IMUInterface* this_ = static_cast<IMUInterface*>(param);

    if (this_ == nullptr)
    {
        vTaskDelete(nullptr);

        return;
    }

    /*Main loop*/
    this_->MainLoop();

    /*Clean up*/
    this_->IMUUnregisterInterrupt();

    this_->handle_ = nullptr;

    vTaskDelete(nullptr);

    return;
}

void IMUInterface::MainLoop() 
{
    while (true)
    {
        if (this->task_stop_.load())
            break;

        switch(this->state_)
        {
            case IMUState::INIT:
                /* Perform initialization tasks, e.g., check IMU status, configure settings */
                // If initialization successful:
                if(this->Init() == ESP_OK)
                {
                    this->state_ = IMUState::RUN;
                }
                // Else if initialization failed:
                else
                {
                    this->state_ = IMUState::ERR;
                }
                break;

            case IMUState::RUN:
                if(this->Run() != ESP_OK)
                {
                    this->state_ = IMUState::ERR;
                }

                break;

            case IMUState::STOP:
                /* Handle stop state, e.g., pause data acquisition */
                break;

            case IMUState::ERR:
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

////////////////////////////////////////////////////////////////////////////////
esp_err_t IMUInterface::Init() 
{
    esp_err_t ret;

    ret = this->imu_.SoftReset();
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(2));

    ret = this->imu_.CheckWhoAmI();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigurePower();
    if (ret != ESP_OK) return ret;
    esp_rom_delay_us(300);

    ret = this->imu_.ConfigureGyro();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureAccel();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureGyro1();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureDLFP();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterrupt();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterrupt1();
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterruptSource();
    if (ret != ESP_OK) return ret;

    this->IMURegisterInterrupt();

    this->read_idx_.store(0);
    this->write_idx_.store(1);
    this->sample_count_ = 0;

    this->imu_read_data_[0] = {};
    this->imu_read_data_[1] = {};
    this->imu_parsed_data_ = {};

    return ESP_OK;
}

esp_err_t IMUInterface::Run() 
{
    uint32_t note = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

    if (this->task_stop_.load())
    {
        return ESP_OK;
    }

    if (note == 0)
    {
        this->imu_notify_timeout_count_++;
    }

    if (note > 1)
    {
        // IMU interrupt가 task 처리보다 빨라서 notification이 누적된 경우
        this->imu_notify_burst_count_ += (note - 1);
    }

    if (note > 0)
    {
        esp_err_t ret = this->ReadIMUData();
        if (ret != ESP_OK) 
        {
            this->imu_read_fail_count_++;
            return ret;
        }

        ret = this->ParseImuData();
        if (ret != ESP_OK) 
        {
            return ret;
        }

        const int64_t now_us = esp_timer_get_time();

        if (this->imu_last_publish_us_ > 0)
        {
            const int64_t gap = now_us - this->imu_last_publish_us_;
            if (gap > this->imu_max_publish_gap_us_)
            {
                this->imu_max_publish_gap_us_ = gap;
            }
        }

        this->imu_last_publish_us_ = now_us;

        ret = this->WriteDataToBuffer();
        if (ret != ESP_OK) 
        {
            return ret;
        }

        this->imu_run_count_++;
    }
    
#if ENABLE_IMU_STATS_LOG
    const int64_t log_now = esp_timer_get_time();

    if (log_now - this->imu_last_log_us_ >= 1000000)
    {
        this->imu_last_log_us_ = log_now;

        ESP_LOGI("IMU_IFACE",
            "IMU stat hz=%lu timeout=%lu burst=%lu read_fail=%lu max_gap_ms=%lld sample=%lu",
            this->imu_run_count_,
            this->imu_notify_timeout_count_,
            this->imu_notify_burst_count_,
            this->imu_read_fail_count_,
            this->imu_max_publish_gap_us_ / 1000,
            this->sample_count_
        );

        this->imu_run_count_ = 0;
        this->imu_notify_timeout_count_ = 0;
        this->imu_notify_burst_count_ = 0;
        this->imu_read_fail_count_ = 0;
        this->imu_max_publish_gap_us_ = 0;
    }
#endif

    return ESP_OK;
}

esp_err_t IMUInterface::Error() 
{
    IMUUnregisterInterrupt();
    vTaskDelay(pdMS_TO_TICKS(100));
    state_ = IMUState::INIT;
    return ESP_OK;
    
}




esp_err_t IMUInterface::ReadIMUData() 
{
    int16_t ax, ay, az, gx, gy, gz, temp;
    esp_err_t ret = imu_.ReadRawImu(ax, ay, az, gx, gy, gz, temp);

    if (ret == ESP_OK)
    {
        this->imu_data_.ax = ax;
        this->imu_data_.ay = ay;
        this->imu_data_.az = az;
        this->imu_data_.gx = gx;
        this->imu_data_.gy = gy;
        this->imu_data_.gz = gz;
        this->imu_data_.temp = temp;
    }
    else
    {
        this->state_ = IMUState::ERR;
    }
    
    return ret;
}

esp_err_t IMUInterface::ParseImuData() 
{

    /* Convert raw IMU data to physical units */
    //accel 2048/4096/8192 LSB/g 
    this->imu_parsed_data_.ax_g = static_cast<float>(this->imu_data_.ax) / 2048.0f;
    this->imu_parsed_data_.ay_g = static_cast<float>(this->imu_data_.ay) / 2048.0f;
    this->imu_parsed_data_.az_g = static_cast<float>(this->imu_data_.az) / 2048.0f;

    //gyro 16.4/32.8/65.6 LSB/dps
    //dps to rad/s: * 0.01745329252
    this->imu_parsed_data_.gx_rad_s = DEG2RAD((static_cast<float>(this->imu_data_.gx)) / 16.4f);
    this->imu_parsed_data_.gy_rad_s = DEG2RAD((static_cast<float>(this->imu_data_.gy)) / 16.4f);
    this->imu_parsed_data_.gz_rad_s = DEG2RAD((static_cast<float>(this->imu_data_.gz)) / 16.4f);

    // Temperature
    this->imu_parsed_data_.temp_c = (static_cast<float>(this->imu_data_.temp)) / 326.8f + 25.0f;

    /* Apply bias corrections */
    this->imu_parsed_data_.ax_g += this->imu_data_bias_.ax_b;
    this->imu_parsed_data_.ay_g += this->imu_data_bias_.ay_b;
    this->imu_parsed_data_.az_g += this->imu_data_bias_.az_b;

    this->imu_parsed_data_.gx_rad_s += this->imu_data_bias_.gx_b;
    this->imu_parsed_data_.gy_rad_s += this->imu_data_bias_.gy_b;
    this->imu_parsed_data_.gz_rad_s += this->imu_data_bias_.gz_b;

    this->imu_parsed_data_.temp_c += this->imu_data_bias_.temp_b;

    /* Apply direction  */
    this->imu_parsed_data_.ax_g *= this->imu_data_dir_.ax_d;
    this->imu_parsed_data_.ay_g *= this->imu_data_dir_.ay_d;
    this->imu_parsed_data_.az_g *= this->imu_data_dir_.az_d;

    this->imu_parsed_data_.gx_rad_s *= this->imu_data_dir_.gx_d;
    this->imu_parsed_data_.gy_rad_s *= this->imu_data_dir_.gy_d;
    this->imu_parsed_data_.gz_rad_s *= this->imu_data_dir_.gz_d;

    this->imu_parsed_data_.temp_c *= this->imu_data_dir_.temp_d;


    return ESP_OK;
}

esp_err_t IMUInterface::WriteDataToBuffer()
{
    // timestamp/sample id는 publish 직전에 기록
    this->imu_parsed_data_.timestamp_us = esp_timer_get_time();
    this->imu_parsed_data_.sample_count = ++this->sample_count_;

    // 1. Write the whole data to write buffer
    const int w = this->write_idx_.load();
    this->imu_read_data_[w] = this->imu_parsed_data_;

    // 2. Publish the completed buffer
    this->read_idx_.store(w);

    // 3. Switch to the next write buffer
    this->write_idx_.store(1 - w);

    return ESP_OK;
}


esp_err_t IMUInterface::GetParsedIMUDpsData(IMU_PARESED_DATA& data) 
{
    IMU_PARESED_DATA data_copy{};

    // if (xSemaphoreTake(this->data_mutex_, pdMS_TO_TICKS(1)) == pdPASS) 
    // {
    //     data_copy = this->imu_read_data_;

    //     xSemaphoreGive(this->data_mutex_);

    //     ret = 0;
    // }

    // Snapshot current published buffer index
    const int idx = this->read_idx_.load();

    data_copy = this->imu_read_data_[idx];

    data_copy.gx_rad_s = RAD2DEG(data_copy.gx_rad_s);
    data_copy.gy_rad_s = RAD2DEG(data_copy.gy_rad_s);
    data_copy.gz_rad_s = RAD2DEG(data_copy.gz_rad_s);

    data = data_copy;

    return ESP_OK;
}

esp_err_t IMUInterface::GetParsedIMURadsData(IMU_PARESED_DATA& data) 
{
    IMU_PARESED_DATA data_copy{};

    // if (xSemaphoreTake(this->data_mutex_, pdMS_TO_TICKS(1)) == pdPASS) 
    // {
    //     data_copy = this->imu_read_data_;

    //     xSemaphoreGive(this->data_mutex_);

    //     ret = 0;
    // }

    // Snapshot current published buffer index
    const int idx = this->read_idx_.load();

    data_copy = this->imu_read_data_[idx];

    data = data_copy;

    return ESP_OK;
}




void IMUInterface::IMUSetDataDir(const IMU_DATA_DIR dir) 
{
    this->imu_data_dir_ = dir;

    return;
}

void IMUInterface::IMUSetDataBias(const IMU_DATA_BIAS& bias)
{
    this->imu_data_bias_ = bias;

    return;
}


IMUInterface::IMUInterface()
{

}

IMUInterface::IMUInterface(board_handles_t* bhandle)
{
    this->bhandle_ = bhandle;
    this->imu_ = ICM42688(bhandle->imu_spi);
}

IMUInterface::~IMUInterface()
{

}



esp_err_t IMUInterface::Initialize(board_handles_t* bhandle) 
{
    esp_err_t ret = this->SetSPIHandle(bhandle);

    if (ret != ESP_OK)
    {
        return ret;
    }   

    if(this->bhandle_ == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->imu_.spi_ = this->bhandle_->imu_spi;

    if(this->imu_.spi_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    IMU_DATA_DIR dir = {IMU_DATA_AX_DIR, IMU_DATA_AY_DIR, IMU_DATA_AZ_DIR, 
                        IMU_DATA_GX_DIR, IMU_DATA_GY_DIR, IMU_DATA_GZ_DIR, 
                        IMU_DATA_TEMP_DIR};
    this->IMUSetDataDir(dir);

    IMU_DATA_BIAS bias = {IMU_DATA_AX_BIAS, IMU_DATA_AY_BIAS, IMU_DATA_AZ_BIAS, 
                            IMU_DATA_GX_BIAS, IMU_DATA_GY_BIAS, IMU_DATA_GZ_BIAS, 
                            IMU_DATA_TEMP_BIAS};
    this->IMUSetDataBias(bias);

    this->StartTask();

    return ESP_OK;
}

esp_err_t IMUInterface::SetSPIHandle(board_handles_t* bhandle)
{
    if (bhandle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->bhandle_ = bhandle;

    return ESP_OK;
}
