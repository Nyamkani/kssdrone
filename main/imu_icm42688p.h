#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/spi_master.h"

#define ICM42688_WHO_AM_I 0x47
#define READ_DATA_BUF_LENGTH 32

enum class IMUCMD : uint8_t
{
    DEVICE_CONFIG             = 0x11,
    WHO_AM_I                  = 0x75,
    PWR_MGMT0                 = 0x4E,
    GYRO_CONFIG0              = 0x4F,
    ACCEL_CONFIG0             = 0x50,
    GYRO_CONFIG1              = 0x51,
    GYRO_ACCEL_CONFIG0        = 0x52,
    ACCEL_CONFIG1             = 0x53,
    INT_CONFIG                = 0x14,
    FIFO_CONFIG               = 0x16,
    TEMP_DATA1                = 0x1D,
    INT_STATUS                = 0x2D,
    FIFO_COUNTH               = 0x2E,
    FIFO_COUNTL               = 0x2F,
    FIFO_DATA                 = 0x30,
    SIGNAL_PATH_RESET         = 0x4B,
    INTF_CONFIG0              = 0x4C,
    INT_CONFIG1               = 0x64,
    INT_SOURCE0               = 0x65,
    FIFO_CONFIG1              = 0x5F,
    FIFO_CONFIG2              = 0x60,
    FIFO_CONFIG3              = 0x61,
    BURST_SPI_READ            = 0x80
};

struct ICM42688Config
{
    uint8_t gyro_config0 = 0x06;
    uint8_t accel_config0 = 0x06;
    uint8_t gyro_config1 = 0x01;
    uint8_t accel_config1 = 0x01;
    uint8_t gyro_accel_config0 = 0x22;

    bool enable_interrupt = true;
    uint8_t int_config = 0x03;
    uint8_t int_config1 = 0x00;
    uint8_t int_source0 = 0x08;
};

static constexpr uint8_t ICM42688_ODR_8KHZ = 0x03;
static constexpr uint8_t ICM42688_ODR_1KHZ = 0x06;

static constexpr uint8_t ICM42688_GYRO_FS_2000DPS = 0x00 << 5;
static constexpr uint8_t ICM42688_ACCEL_FS_16G    = 0x00 << 5;

static constexpr uint8_t ICM42688_FIFO_MODE_BYPASS = 0x00;
static constexpr uint8_t ICM42688_FIFO_MODE_STREAM = 0x40;

static constexpr uint8_t ICM42688_FIFO_ACCEL_EN = 0x01;
static constexpr uint8_t ICM42688_FIFO_GYRO_EN  = 0x02;
static constexpr uint8_t ICM42688_FIFO_TEMP_EN  = 0x04;

static constexpr uint8_t ICM42688_INT1_FIFO_THS = 0x04;
static constexpr uint8_t ICM42688_INT1_UI_DRDY  = 0x08;

static constexpr uint8_t ICM42688_FIFO_PACKET_SIZE = 16;


class ICM42688
{
    public:
        ICM42688();
        explicit ICM42688(spi_device_handle_t spi);
        ~ICM42688() = default;

        esp_err_t SetSPIHandle(spi_device_handle_t spi);
        esp_err_t Initialize(spi_device_handle_t spi);

        esp_err_t WriteRegister(uint8_t reg, uint8_t value);
        esp_err_t ReadRegister(uint8_t reg, uint8_t *buf);
        esp_err_t ReadRegisters(uint8_t reg, uint8_t *buf, size_t len);

        esp_err_t SoftReset();
        esp_err_t CheckWhoAmI();
        esp_err_t ConfigurePower();
        esp_err_t ConfigureGyro(const uint8_t val);
        esp_err_t ConfigureGyro1(const uint8_t val);
        esp_err_t ConfigureAccel(const uint8_t val);
        esp_err_t ConfigureAccel1(const uint8_t val);
        esp_err_t ConfigureDLFP(const uint8_t val);
        esp_err_t ConfigureInterrupt(const uint8_t val);
        esp_err_t ConfigureInterrupt1(const uint8_t val);
        esp_err_t ConfigureInterruptSource(const uint8_t val);

        esp_err_t Configure(const ICM42688Config& config);

        esp_err_t ReadRawImu(int16_t& ax, int16_t& ay, int16_t& az,
                            int16_t& gx, int16_t& gy, int16_t& gz,
                            int16_t& temp, bool clear_interrupt_status = false);

        esp_err_t FlushFifo();
        esp_err_t ConfigureFifo(uint8_t fifo_config,uint8_t fifo_config1, uint16_t watermark_bytes);
        esp_err_t ReadFifoCount(uint16_t& count);
        esp_err_t ReadFifoBytes(uint8_t* buf, size_t len);
        esp_err_t ClearInterruptStatus(uint8_t* status = nullptr);

    public:
        spi_device_handle_t spi_ = nullptr;
};