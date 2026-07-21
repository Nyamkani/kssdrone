#include "imu_icm42688p.h"

ICM42688::ICM42688()
    : spi_(nullptr)
{
}   


ICM42688::ICM42688(spi_device_handle_t spi)
    : spi_(spi)
{
}

esp_err_t ICM42688::SetSPIHandle(spi_device_handle_t spi)
{
    if (spi == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->spi_ = spi;

    return ESP_OK;
}


esp_err_t ICM42688::Initialize(spi_device_handle_t spi)
{
    esp_err_t ret = SetSPIHandle(spi);
    if (ret != ESP_OK)
    {
        return ret;
    }
    
    return ESP_OK;
}



esp_err_t ICM42688::WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t send_buf[2] = {
        static_cast<uint8_t>(reg & 0x7F),
        value
    };

    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = send_buf;
    trans.rx_buffer = nullptr;

    return spi_device_transmit(spi_, &trans);
}

esp_err_t ICM42688::ReadRegister(uint8_t reg, uint8_t *buf)
{
    if (buf == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t send_buf[2] = {
        static_cast<uint8_t>(reg | static_cast<uint8_t>(IMUCMD::BURST_SPI_READ)),
        0x00
    };
    uint8_t read_buf[2] = {};

    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = send_buf;
    trans.rx_buffer = read_buf;

    esp_err_t ret = spi_device_transmit(spi_, &trans);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *buf = read_buf[1];
    return ESP_OK;
}

esp_err_t ICM42688::ReadRegisters(uint8_t reg, uint8_t *buf, size_t len)
{
    if (buf == nullptr || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((len + 1) > READ_DATA_BUF_LENGTH)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t send_buf[READ_DATA_BUF_LENGTH] = {};
    uint8_t read_buf[READ_DATA_BUF_LENGTH] = {};

    send_buf[0] = static_cast<uint8_t>(
        reg | static_cast<uint8_t>(IMUCMD::BURST_SPI_READ)
    );

    spi_transaction_t trans = {};
    trans.length = 8 * (len + 1);
    trans.tx_buffer = send_buf;
    trans.rx_buffer = read_buf;

    esp_err_t ret = spi_device_transmit(spi_, &trans);
    if (ret != ESP_OK)
    {
        return ret;
    }

    for (size_t i = 0; i < len; ++i)
    {
        buf[i] = read_buf[i + 1];
    }

    return ESP_OK;
}

esp_err_t ICM42688::SoftReset()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::DEVICE_CONFIG), 0x01);
}

esp_err_t ICM42688::CheckWhoAmI()
{
    uint8_t buf = 0;
    esp_err_t ret = ReadRegister(static_cast<uint8_t>(IMUCMD::WHO_AM_I), &buf);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (buf != ICM42688_WHO_AM_I)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t ICM42688::ConfigurePower()
{
    WriteRegister(0x76, 0x00); 

    return WriteRegister(static_cast<uint8_t>(IMUCMD::PWR_MGMT0), 0x0F);
}

esp_err_t ICM42688::ConfigureGyro()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::GYRO_CONFIG0), 0x06); //0x67
}

esp_err_t ICM42688::ConfigureGyro1()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::GYRO_CONFIG1), 0x01); //0x67
}

esp_err_t ICM42688::ConfigureAccel()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::ACCEL_CONFIG0), 0x06);
}

esp_err_t ICM42688::ConfigureDLFP()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::GYRO_ACCEL_CONFIG0), 0x22); 
}

esp_err_t ICM42688::ConfigureInterrupt()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::INT_CONFIG), 0x03); //0x03
    // uint8_t val = 0;

    // esp_err_t ret = WriteRegister(static_cast<uint8_t>(IMUCMD::INT_CONFIG), 0x01);
    // if (ret != ESP_OK) return ret;

    // return ret;
}

esp_err_t ICM42688::ConfigureInterrupt1()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::INT_CONFIG1), 0x00); //0x01
}

esp_err_t ICM42688::ConfigureInterruptSource()
{
    return WriteRegister(static_cast<uint8_t>(IMUCMD::INT_SOURCE0), 0x08);
    // WriteRegister(0x63, 0x08);

    //     uint8_t int_clear = 0;

    // esp_err_t ret = ReadRegister(0x2d, &int_clear);
    // if (ret != ESP_OK)
    // {
    //     return ret;
    // }


    // return ret;
}

esp_err_t ICM42688::ReadRawImu(int16_t& ax, int16_t& ay, int16_t& az,
                               int16_t& gx, int16_t& gy, int16_t& gz,
                               int16_t& temp)
{
    uint8_t buf[14] = {};

    uint8_t int_clear = 0;

    esp_err_t ret = ReadRegisters(static_cast<uint8_t>(IMUCMD::TEMP_DATA1), buf, 14);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = ReadRegister(0x2d, &int_clear);
    if (ret != ESP_OK)
    {
        return ret;
    }

    temp = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    ax   = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    ay   = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    az   = static_cast<int16_t>((buf[6] << 8) | buf[7]);
    gx   = static_cast<int16_t>((buf[8] << 8) | buf[9]);
    gy   = static_cast<int16_t>((buf[10] << 8) | buf[11]);
    gz   = static_cast<int16_t>((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

