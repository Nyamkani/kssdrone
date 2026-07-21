/*Configuration of FreeRTOS*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*ESP-IDF specific headers*/
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"


#include "gpio_config.h"
#include "imu_icm42688p.h"
#include "shared_snapshot.h"

#include <cmath>
#include <atomic>

#define IMU_DATA_AX_DIR 1.0f
#define IMU_DATA_AY_DIR 1.0f
#define IMU_DATA_AZ_DIR 1.0f
#define IMU_DATA_GX_DIR 1.0f
#define IMU_DATA_GY_DIR 1.0f
#define IMU_DATA_GZ_DIR 1.0f
#define IMU_DATA_TEMP_DIR 1.0f

#define IMU_DATA_AX_BIAS 0.0f
#define IMU_DATA_AY_BIAS 0.0f
#define IMU_DATA_AZ_BIAS 0.0f
#define IMU_DATA_GX_BIAS 0.0f
#define IMU_DATA_GY_BIAS 0.0f
#define IMU_DATA_GZ_BIAS 0.0f
#define IMU_DATA_TEMP_BIAS 0.0f

#define DEG2RAD(x) ((x) * 0.01745329252f)
#define RAD2DEG(x) ((x) * 57.2957795131f)


#ifndef IMU_FIFO_EXTRA_READ_SAMPLES
#define IMU_FIFO_EXTRA_READ_SAMPLES 4
#endif


//log
#define ENABLE_IMU_STATS_LOG 0


#define IMU_READ_MODE_DIRECT              0
#define IMU_READ_MODE_INTERRUPT_LATEST    1
#define IMU_READ_MODE_FIFO_INTERRUPT      2

#ifndef IMU_READ_MODE
#define IMU_READ_MODE IMU_READ_MODE_INTERRUPT_LATEST
#endif

#ifndef IMU_CLEAR_INT_STATUS_AFTER_READ
#define IMU_CLEAR_INT_STATUS_AFTER_READ 1
#endif

/*
 * INTERRUPT_LATEST 모드에서 사용.
 *
 * ICM ODR register is set to 8kHz,
 * but observed UI_DRDY interrupt rate is about 4kHz.
 * Use DIV=4 to publish IMU samples to control loop at ~1kHz.
 *
 */
#ifndef IMU_INTERRUPT_READ_DIV
#define IMU_INTERRUPT_READ_DIV 8   
#endif
/*
 * FIFO 모드에서 사용.
 *
 * ODR 8kHz 기준:
 * watermark 8 samples -> 1kHz interrupt
 * watermark 4 samples -> 2kHz interrupt
 */
#ifndef IMU_FIFO_WATERMARK_SAMPLES
#define IMU_FIFO_WATERMARK_SAMPLES 8
#endif


enum class IMUState
{
    INIT = 0,
    RUN = 1, 
    STOP = 2,   
    ERR = 10,

};
    
struct IMU_RAW_DATA
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t temp;
};

struct IMU_PARESED_DATA
{
    float ax_g;
    float ay_g;
    float az_g;

    float gx_rad_s;
    float gy_rad_s;
    float gz_rad_s;

    float temp_c;

    int64_t timestamp_us;
    uint32_t sample_count;

};

struct IMU_DATA_DIR
{
    float ax_d = 1.0f;
    float ay_d = 1.0f;
    float az_d = 1.0f;

    float gx_d = 1.0f;
    float gy_d = 1.0f;
    float gz_d = 1.0f;

    float temp_d = 1.0f;
};

struct IMU_DATA_BIAS
{
    float ax_b = 0.0f;
    float ay_b = 0.0f;
    float az_b = 0.0f;

    float gx_b = 0.0f;
    float gy_b = 0.0f;
    float gz_b = 0.0f;

    float temp_b = 0.0f;
};

struct ImuControlSample
{
    IMU_PARESED_DATA data{};

    uint32_t seq = 0;
    int64_t timestamp_us = 0;

    /*
     * FIFO 모드에서는 한 번에 몇 개 sample을 처리했는지 기록.
     * DIRECT/INTERRUPT_LATEST에서는 1.
     */
    uint16_t sample_count = 1;

    bool valid = false;
};


class IMUInterface 
{
    public:
        TaskHandle_t handle_ = nullptr;
        // SemaphoreHandle_t  data_mutex_ = nullptr;
        board_handles_t* bhandle_ = nullptr;

    private:
        std::atomic<bool> task_stop_{false};
        IMUState state_{IMUState::INIT};
        ICM42688 imu_{nullptr};

        IMU_RAW_DATA imu_data_{};
        IMU_PARESED_DATA imu_parsed_data_{};
   
        IMU_DATA_DIR imu_data_dir_{};
        IMU_DATA_BIAS imu_data_bias_{};

        SharedSnapshot<IMU_PARESED_DATA> imu_snapshot_;

        uint32_t sample_count_ = 0;

        uint32_t imu_run_count_ = 0;
        uint32_t imu_notify_timeout_count_ = 0;
        uint32_t imu_notify_burst_count_ = 0;
        uint32_t imu_read_fail_count_ = 0;

        int64_t imu_last_publish_us_ = 0;
        int64_t imu_max_publish_gap_us_ = 0;
        int64_t imu_last_log_us_ = 0;

        volatile uint32_t imu_isr_count_ = 0;
        uint32_t imu_isr_count_log_prev_ = 0;
        
    public:
        /*Initialization*/
        esp_err_t Initialize(board_handles_t* bhandle);
        esp_err_t StartTask();

        /*Deinitialization*/
        void StopTaskRequest();
        void MainLoop();

        void IMURegisterInterrupt();
        void IMUUnregisterInterrupt();



        esp_err_t GetParsedIMUDpsData(IMU_PARESED_DATA& data);
        esp_err_t GetParsedIMURadsData(IMU_PARESED_DATA& data);
        esp_err_t GetParsedIMURadsDataWithFrame(SharedSnapshotFrame<IMU_PARESED_DATA>& frame);
        esp_err_t GetControlImuFrame(SharedSnapshotFrame<IMU_PARESED_DATA>& frame);

        esp_err_t RunInterruptLatestMode();
        esp_err_t RunFifoInterruptMode();

        void IMUSetDataDir(const IMU_DATA_DIR dir);
        void IMUSetDataBias(const IMU_DATA_BIAS& bias); 

        IMUState GetState() const { return this->state_; } 

        bool OnImuInterruptFromISR(BaseType_t* high_task_wakeup);

 
        IMUInterface(const IMUInterface&) = delete;
        IMUInterface& operator=(const IMUInterface&) = delete;
        IMUInterface(board_handles_t* bhandle);
        IMUInterface();
        virtual ~IMUInterface();

    private:
        esp_err_t Init();
        esp_err_t Run();
        esp_err_t Error();

        esp_err_t SetSPIHandle(board_handles_t* bhandle);

        static void MainTask(void* param);
        esp_err_t ReadIMUData();
        esp_err_t ParseImuData();
        esp_err_t WriteDataToBuffer();
        esp_err_t WriteDataToBuffer(uint32_t frame_sample_count);

        esp_err_t ConfigureBaseSensor();
        esp_err_t ConfigureInterruptLatestMode();
        esp_err_t ConfigureFifoInterruptMode();
        void ResetRuntimeStats();
        esp_err_t SeedInitialSnapshot();

};  



