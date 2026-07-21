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
    INT_CONFIG                = 0x14,
    INT_CONFIG1               = 0x64,
    INT_SOURCE0               = 0x65,
    TEMP_DATA1                = 0x1D,
    BURST_SPI_READ            = 0x80
};

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
        esp_err_t ConfigureGyro();
        esp_err_t ConfigureGyro1();
        esp_err_t ConfigureAccel();
        esp_err_t ConfigureDLFP();
        esp_err_t ConfigureInterrupt();
        esp_err_t ConfigureInterrupt1();
        esp_err_t ConfigureInterruptSource();

        esp_err_t ReadRawImu(int16_t& ax, int16_t& ay, int16_t& az,
                            int16_t& gx, int16_t& gy, int16_t& gz,
                            int16_t& temp);

    public:
        spi_device_handle_t spi_ = nullptr;
};