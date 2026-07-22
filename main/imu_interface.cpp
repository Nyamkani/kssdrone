#include "imu_interface.h"
#include "pins.h"
#include "driver/gpio.h"

static const char* TAG = "imu_interface";

static void IRAM_ATTR imu_int_isr(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;

    IMUInterface* this_ = static_cast<IMUInterface*>(arg);

    if (this_ != nullptr && this_->handle_ != nullptr)
    {
        if (this_->OnImuInterruptFromISR(&high_task_wakeup))
        {
            if (high_task_wakeup)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

bool IRAM_ATTR IMUInterface::OnImuInterruptFromISR(
    BaseType_t* high_task_wakeup)
{
    this->imu_isr_count_ += 1;

#if IMU_READ_MODE == IMU_READ_MODE_INTERRUPT_LATEST
    if ((this->imu_isr_count_ % IMU_INTERRUPT_READ_DIV) != 0)
    {
        return false;
    }
#endif

    vTaskNotifyGiveFromISR(this->handle_, high_task_wakeup);
    return true;
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

esp_err_t IMUInterface::Initialize(board_handles_t* bhandle)
{
    esp_err_t ret = this->SetSPIHandle(bhandle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (this->bhandle_ == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->imu_.spi_ = this->bhandle_->imu_spi;
    if (this->imu_.spi_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    IMU_DATA_DIR dir = {
        IMU_DATA_AX_DIR, IMU_DATA_AY_DIR, IMU_DATA_AZ_DIR,
        IMU_DATA_GX_DIR, IMU_DATA_GY_DIR, IMU_DATA_GZ_DIR,
        IMU_DATA_TEMP_DIR
    };
    this->IMUSetDataDir(dir);

    IMU_DATA_BIAS bias = {
        IMU_DATA_AX_BIAS, IMU_DATA_AY_BIAS, IMU_DATA_AZ_BIAS,
        IMU_DATA_GX_BIAS, IMU_DATA_GY_BIAS, IMU_DATA_GZ_BIAS,
        IMU_DATA_TEMP_BIAS
    };
    this->IMUSetDataBias(bias);

#if IMU_READ_MODE == IMU_READ_MODE_DIRECT
    ret = this->Init();
    if (ret != ESP_OK)
    {
        this->state_ = IMUState::ERR;
        return ret;
    }

    this->state_ = IMUState::RUN;
    return ESP_OK;
#else
    return this->StartTask();
#endif
}


esp_err_t IMUInterface::StartTask()
{
#if IMU_READ_MODE == IMU_READ_MODE_DIRECT
    /*
     * DIRECT mode:
     * MainLoop에서 직접 IMU를 읽으므로 IMUTask를 만들지 않는다.
     */
    this->handle_ = nullptr;
    this->task_stop_.store(false);
    return ESP_OK;
#else
    this->task_stop_.store(false);

    BaseType_t ret = xTaskCreatePinnedToCore(
        MainTask,
        "IMUTask",
        4096,
        this,
        7,
        &this->handle_,
        1
    );

    if (ret != pdPASS)
    {
        this->handle_ = nullptr;
        return ESP_FAIL;
    }

    return ESP_OK;
#endif
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
    esp_err_t ret = ESP_OK;

    ret = this->imu_.SoftReset();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "IMU SoftReset failed with error: %d", ret);
        
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ret = this->imu_.CheckWhoAmI();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "IMU CheckWhoAmI failed with error: %d", ret);
        return ret;
    }

    ret = this->imu_.ConfigurePower();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "IMU ConfigurePower failed with error: %d", ret);
        return ret;
    }

    esp_rom_delay_us(300);

    ret = this->ConfigureBaseSensor();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "IMU ConfigureBaseSensor failed with error: %d", ret);
        return ret;
    }

#if IMU_READ_MODE == IMU_READ_MODE_FIFO_INTERRUPT
    ret = this->ConfigureFifoInterruptMode();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "IMU ConfigureFifoInterruptMode failed with error: %d", ret);
        return ret;
    }
#elif IMU_READ_MODE == IMU_READ_MODE_INTERRUPT_LATEST
    ret = this->ConfigureInterruptLatestMode();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "IMU ConfigureInterruptLatestMode failed with error: %d", ret);
        return ret;
    }   
#else
    // DIRECT mode: no interrupt
#endif

    this->ResetRuntimeStats();

#if IMU_READ_MODE == IMU_READ_MODE_FIFO_INTERRUPT
    ret = this->SeedInitialSnapshot();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU SeedInitialSnapshot failed with error: %d", ret);
        return ret;
    }
#endif

    return ESP_OK;
}

esp_err_t IMUInterface::ConfigureBaseSensor()
{
    esp_err_t ret = ESP_OK;

    ICM42688Config config{};

    config.gyro_config0 =
        ICM42688_GYRO_FS_2000DPS |
        ICM42688_ODR_8KHZ;

    config.accel_config0 =
        ICM42688_ACCEL_FS_16G |
        ICM42688_ODR_8KHZ;

    config.accel_config1 = 0x00;
    config.gyro_config1 = 0x01;
    config.gyro_accel_config0 = 0x00;  // or 0x22

    config.enable_interrupt = false;

    ret = this->imu_.Configure(config);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}
                       
esp_err_t IMUInterface::ConfigureInterruptLatestMode()
{
    esp_err_t ret = ESP_OK;

    ret = this->imu_.ConfigureInterrupt(0x03);
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterrupt1(0x60);
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterruptSource(0x08);
    if (ret != ESP_OK) return ret;

    uint8_t int_status = 0;
    ret = this->imu_.ClearInterruptStatus(&int_status);
    if (ret != ESP_OK) return ret;

    this->IMURegisterInterrupt();

    return ESP_OK;
}

esp_err_t IMUInterface::ConfigureFifoInterruptMode()
{
    esp_err_t ret = ESP_OK;

    constexpr uint16_t fifo_watermark_bytes =
        ICM42688_FIFO_PACKET_SIZE * IMU_FIFO_WATERMARK_SAMPLES;

    ret = this->imu_.ConfigureFifo(
        ICM42688_FIFO_MODE_STREAM,
        ICM42688_FIFO_ACCEL_EN |
        ICM42688_FIFO_GYRO_EN |
        ICM42688_FIFO_TEMP_EN,
        fifo_watermark_bytes
    );
    if (ret != ESP_OK) return ret;

    uint8_t int_status = 0;
    ret = this->imu_.ClearInterruptStatus(&int_status);
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterrupt(0x03);
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterrupt1(0x60);
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ConfigureInterruptSource(0x04);
    if (ret != ESP_OK) return ret;

    ret = this->imu_.ClearInterruptStatus(&int_status);
    if (ret != ESP_OK) return ret;

    this->IMURegisterInterrupt();

    return ESP_OK;
}

void IMUInterface::ResetRuntimeStats()
{
    this->sample_count_ = 0;
    this->imu_parsed_data_ = {};

    this->imu_last_publish_us_ = 0;
    this->imu_max_publish_gap_us_ = 0;

    this->imu_run_count_ = 0;
    this->imu_notify_timeout_count_ = 0;
    this->imu_notify_burst_count_ = 0;
    this->imu_read_fail_count_ = 0;

    this->imu_isr_count_ = 0;
    this->imu_isr_count_log_prev_ = 0;
    this->imu_last_log_us_ = esp_timer_get_time();
}

esp_err_t IMUInterface::Run()
{
    esp_err_t ret = ESP_OK;
#if IMU_READ_MODE == IMU_READ_MODE_DIRECT
    /*
     * DIRECT mode에서는 StartTask()가 task를 만들지 않으므로
     * 이 함수는 호출되지 않아야 한다.
     */
#elif IMU_READ_MODE == IMU_READ_MODE_INTERRUPT_LATEST
    ret = this->RunInterruptLatestMode();
#elif IMU_READ_MODE == IMU_READ_MODE_FIFO_INTERRUPT
    ret = this->RunFifoInterruptMode();
#endif
    return ret;
}


esp_err_t IMUInterface::RunInterruptLatestMode()
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
        /*
         * IMU interrupt가 task 처리보다 빨라서 notification이 누적된 경우.
         * binary가 아니라 counting notify처럼 동작하므로 note가 2 이상일 수 있다.
         */
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
        const uint32_t isr_now = this->imu_isr_count_;
        const uint32_t isr_hz = isr_now - this->imu_isr_count_log_prev_;
        this->imu_isr_count_log_prev_ = isr_now;

        this->imu_last_log_us_ = log_now;

        ESP_LOGI("IMU_IFACE",
            "IMU stat mode=int_latest isr=%lu hz=%lu timeout=%lu burst=%lu read_fail=%lu max_gap_ms=%lld sample=%lu",
            isr_hz,
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

esp_err_t IMUInterface::RunFifoInterruptMode()
{
    uint32_t note = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

    if (this->task_stop_.load())
    {
        return ESP_OK;
    }

    if (note == 0)
    {
        this->imu_notify_timeout_count_++;
        return ESP_OK;
    }

    if (note > 1)
    {
        this->imu_notify_burst_count_ += (note - 1);
    }

    uint16_t fifo_count = 0;
    esp_err_t ret = this->imu_.ReadFifoCount(fifo_count);
    if (ret != ESP_OK)
    {
        this->imu_read_fail_count_++;

        uint8_t int_status = 0;
        this->imu_.ClearInterruptStatus(&int_status);

        return ret;
    }

    const uint16_t packet_count =
        fifo_count / ICM42688_FIFO_PACKET_SIZE;

    if (packet_count == 0)
    {
        uint8_t int_status = 0;
        this->imu_.ClearInterruptStatus(&int_status);
        return ESP_OK;
    }

    const uint16_t read_packets =
        std::min<uint16_t>(packet_count, IMU_FIFO_WATERMARK_SAMPLES + IMU_FIFO_EXTRA_READ_SAMPLES);

    const size_t read_len =
        static_cast<size_t>(read_packets) * ICM42688_FIFO_PACKET_SIZE;

    uint8_t fifo_buf[ICM42688_FIFO_PACKET_SIZE * (IMU_FIFO_WATERMARK_SAMPLES + IMU_FIFO_EXTRA_READ_SAMPLES)] = {};

    ret = this->imu_.ReadFifoBytes(fifo_buf, read_len);
    if (ret != ESP_OK)
    {
        this->imu_read_fail_count_++;

        uint8_t int_status = 0;
        this->imu_.ClearInterruptStatus(&int_status);

        return ret;
    }

    const uint8_t* p =
        &fifo_buf[(read_packets - 1) * ICM42688_FIFO_PACKET_SIZE];

    auto be16 = [](const uint8_t* b) -> int16_t
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(b[0]) << 8) |
            static_cast<uint16_t>(b[1])
        );
    };

    this->imu_data_.ax = be16(&p[1]);
    this->imu_data_.ay = be16(&p[3]);
    this->imu_data_.az = be16(&p[5]);

    this->imu_data_.gx = be16(&p[7]);
    this->imu_data_.gy = be16(&p[9]);
    this->imu_data_.gz = be16(&p[11]);

    this->imu_data_.temp = 0;

    ret = this->ParseImuData();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->WriteDataToBuffer(read_packets);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t int_status = 0;
    ret = this->imu_.ClearInterruptStatus(&int_status);
    if (ret != ESP_OK)
    {
        this->imu_read_fail_count_++;
        return ret;
    }

    this->imu_run_count_++;


#if ENABLE_IMU_STATS_LOG
    const int64_t log_now = esp_timer_get_time();

    if (log_now - this->imu_last_log_us_ >= 1000000)
    {
        const uint32_t isr_now = this->imu_isr_count_;
        const uint32_t isr_hz = isr_now - this->imu_isr_count_log_prev_;
        this->imu_isr_count_log_prev_ = isr_now;

        this->imu_last_log_us_ = log_now;

        ESP_LOGI("IMU_IFACE",
            "IMU stat mode=fifo isr=%lu hz=%lu timeout=%lu burst=%lu read_fail=%lu max_gap_ms=%lld sample=%lu fifo_count=%u packets=%u read_packets=%u",
            isr_hz,
            this->imu_run_count_,
            this->imu_notify_timeout_count_,
            this->imu_notify_burst_count_,
            this->imu_read_fail_count_,
            this->imu_max_publish_gap_us_ / 1000,
            this->sample_count_,
            fifo_count,
            packet_count,
            read_packets
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


// esp_err_t IMUInterface::Run() 
// {
//     uint32_t note = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

//     if (this->task_stop_.load())
//     {
//         return ESP_OK;
//     }

//     if (note == 0)
//     {
//         this->imu_notify_timeout_count_++;
//     }

//     if (note > 1)
//     {
//         // IMU interrupt가 task 처리보다 빨라서 notification이 누적된 경우
//         this->imu_notify_burst_count_ += (note - 1);
//     }

//     if (note > 0)
//     {
//         esp_err_t ret = this->ReadIMUData();
//         if (ret != ESP_OK) 
//         {
//             this->imu_read_fail_count_++;
//             return ret;
//         }

//         ret = this->ParseImuData();
//         if (ret != ESP_OK) 
//         {
//             return ret;
//         }

//         const int64_t now_us = esp_timer_get_time();

//         if (this->imu_last_publish_us_ > 0)
//         {
//             const int64_t gap = now_us - this->imu_last_publish_us_;
//             if (gap > this->imu_max_publish_gap_us_)
//             {
//                 this->imu_max_publish_gap_us_ = gap;
//             }
//         }

//         this->imu_last_publish_us_ = now_us;

//         ret = this->WriteDataToBuffer();
//         if (ret != ESP_OK) 
//         {
//             return ret;
//         }

//         this->imu_run_count_++;
//     }
    
// #if ENABLE_IMU_STATS_LOG
//     const int64_t log_now = esp_timer_get_time();

//     if (log_now - this->imu_last_log_us_ >= 1000000)
//     {
//         this->imu_last_log_us_ = log_now;

//         ESP_LOGI("IMU_IFACE",
//             "IMU stat hz=%lu timeout=%lu burst=%lu read_fail=%lu max_gap_ms=%lld sample=%lu",
//             this->imu_run_count_,
//             this->imu_notify_timeout_count_,
//             this->imu_notify_burst_count_,
//             this->imu_read_fail_count_,
//             this->imu_max_publish_gap_us_ / 1000,
//             this->sample_count_
//         );

//         this->imu_run_count_ = 0;
//         this->imu_notify_timeout_count_ = 0;
//         this->imu_notify_burst_count_ = 0;
//         this->imu_read_fail_count_ = 0;
//         this->imu_max_publish_gap_us_ = 0;
//     }
// #endif

//     return ESP_OK;
// }

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

#if IMU_READ_MODE == IMU_READ_MODE_DIRECT
    constexpr bool clear_interrupt_status = false;
#else
    constexpr bool clear_interrupt_status =
        (IMU_CLEAR_INT_STATUS_AFTER_READ != 0);
#endif

    esp_err_t ret = imu_.ReadRawImu(
        ax, ay, az,
        gx, gy, gz,
        temp,
        clear_interrupt_status
    );

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
    return this->WriteDataToBuffer(1);
}

esp_err_t IMUInterface::WriteDataToBuffer(uint32_t frame_sample_count)
{
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

    IMU_PARESED_DATA sample = this->imu_parsed_data_;
    sample.timestamp_us = now_us;
    sample.sample_count = ++this->sample_count_;

    this->imu_snapshot_.Write(sample, now_us, frame_sample_count);

    return ESP_OK;
}

esp_err_t IMUInterface::GetParsedIMUDpsData(IMU_PARESED_DATA& data)
{
    SharedSnapshotFrame<IMU_PARESED_DATA> frame{};

    const esp_err_t ret = this->GetControlImuFrame(frame);
    if (ret != ESP_OK)
    {
        return ret;
    }

    IMU_PARESED_DATA data_copy = frame.data;

    data_copy.gx_rad_s = RAD2DEG(data_copy.gx_rad_s);
    data_copy.gy_rad_s = RAD2DEG(data_copy.gy_rad_s);
    data_copy.gz_rad_s = RAD2DEG(data_copy.gz_rad_s);

    data = data_copy;
    return ESP_OK;
}

esp_err_t IMUInterface::GetParsedIMURadsData(IMU_PARESED_DATA& data)
{
    SharedSnapshotFrame<IMU_PARESED_DATA> frame{};

    const esp_err_t ret = this->GetParsedIMURadsDataWithFrame(frame);
    if (ret != ESP_OK)
    {
        return ret;
    }

    data = frame.data;
    return ESP_OK;
}

esp_err_t IMUInterface::GetParsedIMURadsDataWithFrame(SharedSnapshotFrame<IMU_PARESED_DATA>& frame)
{
    return this->GetControlImuFrame(frame);
}



esp_err_t IMUInterface::GetControlImuFrame(SharedSnapshotFrame<IMU_PARESED_DATA>& frame)
{
#if IMU_READ_MODE == IMU_READ_MODE_DIRECT

    esp_err_t ret = this->ReadIMUData();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->ParseImuData();
    if (ret != ESP_OK)
    {
        return ret;
    }

    const int64_t now_us = esp_timer_get_time();
    const uint32_t seq = ++this->sample_count_;

    this->imu_parsed_data_.timestamp_us = now_us;
    this->imu_parsed_data_.sample_count = seq;

    frame.data = this->imu_parsed_data_;
    frame.seq = seq;
    frame.timestamp_us = now_us;
    frame.sample_count = 1;
    frame.valid = true;

    return ESP_OK;

#else

    if (!this->imu_snapshot_.Read(frame))
    {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;

#endif
}




esp_err_t IMUInterface::SeedInitialSnapshot()
{
    esp_err_t ret = this->ReadIMUData();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = this->ParseImuData();
    if (ret != ESP_OK)
    {
        return ret;
    }

    return this->WriteDataToBuffer(1);
}




////////////////////////////////////////////////////////////

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

esp_err_t IMUInterface::SetSPIHandle(board_handles_t* bhandle)
{
    if (bhandle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->bhandle_ = bhandle;

    return ESP_OK;
}

