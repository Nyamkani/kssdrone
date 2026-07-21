#pragma once

#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    spi_host_device_t spi_host;
    spi_device_handle_t imu_spi;
} board_handles_t;

/**
 * @brief LED / BUZZER GPIO Initialization
 */
esp_err_t BoardInitGPIOOutputs(void);

/**
 * @brief IMU INT GPIO Initialization
 */
esp_err_t BoardInitIMUIntGPIO(void);

/**
 * @brief SPI Bus Initialization and IMU SPI Device Registration
 *
 * @param handles SPI handle storage structure
 */
esp_err_t BoardInitSPI(board_handles_t *handles);

/**
 * @brief BAT ADC Initialization
 */
esp_err_t BoardInitBatteryADC(void);

/**
 * @brief IMU Interrupt Service Installation
 *
 * @param isr_handler GPIO ISR Handler
 * @param arg ISR Argument
 */
esp_err_t BoardInstallIMUISR(gpio_isr_t isr_handler, void *arg);

/**
 * @brief Read raw ADC value from battery voltage divider
 *
 * @param raw Pointer to store the raw ADC value
 */ 
esp_err_t BoardReadBatteryADCRaw(int *raw);

/**
 * @brief Board Initialization (GPIO, ADC, SPI)
 * :
 * 1. GPIO output
 * 2. BAT ADC
 * 3. SPI
 * 4. IMU INT GPIO
 *
 * @param handles SPI handle storage structure
 */
esp_err_t BoardInitAll(board_handles_t *handles);

/**
 * @brief Set the buzzer and LED state
 *
 * @param on true to turn on, false to turn off
 */
void BoardSetBuzzerLed(bool on);

//uart
esp_err_t BoardUartInit();

QueueHandle_t GetElrsUartEventQueue();

#ifdef __cplusplus
}
#endif