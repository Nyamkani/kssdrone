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

#define ENABLE_IMU_STATS_LOG 0


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
        IMU_PARESED_DATA imu_read_data_[2]{};
   
        IMU_DATA_DIR imu_data_dir_{};
        IMU_DATA_BIAS imu_data_bias_{};

        // read_idx_ points to the latest completed packet.
        // write_idx_ is the buffer index used by producer task.
        std::atomic<int> read_idx_{0};
        std::atomic<int> write_idx_{1};

        uint32_t sample_count_ = 0;

        uint32_t imu_run_count_ = 0;
        uint32_t imu_notify_timeout_count_ = 0;
        uint32_t imu_notify_burst_count_ = 0;
        uint32_t imu_read_fail_count_ = 0;

        int64_t imu_last_publish_us_ = 0;
        int64_t imu_max_publish_gap_us_ = 0;
        int64_t imu_last_log_us_ = 0;



        
    public:
        /*Initialization*/
        esp_err_t Initialize(board_handles_t* bhandle);
        void StartTask();

        /*Deinitialization*/
        void StopTaskRequest();
        void MainLoop();

        void IMURegisterInterrupt();
        void IMUUnregisterInterrupt();



        esp_err_t GetParsedIMUDpsData(IMU_PARESED_DATA& data);
        esp_err_t GetParsedIMURadsData(IMU_PARESED_DATA& data);

        void IMUSetDataDir(const IMU_DATA_DIR dir);
        void IMUSetDataBias(const IMU_DATA_BIAS& bias); 


        IMUState GetState() const { return this->state_; } 


 
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



};  



