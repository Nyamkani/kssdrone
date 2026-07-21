#pragma once

#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RMT_TX_CHANNEL_CONFIG_MEM_BLOCK_SYMBOLS 48  //32 ,48, 64
#define RMT_TX_CHANNEL_CONFIG_QUEUE_DEPTH 2  //2,4,6s
#define DSHOT_RESOL_HZ 40*1000*1000 // 40MHz resolution for DSHOT (25ns per tick)

class Motor
{
    public:
        Motor() = default;
        virtual ~Motor() = default;

        esp_err_t Initialize(gpio_num_t pin);
        esp_err_t SendThrottle(uint16_t throttle, bool telemetry = false);
        esp_err_t WaitDone(TickType_t timeout_ticks);
        //esp_err_t SendRawPacket(uint16_t packet);
        //esp_err_t Stop();

    private:
        uint16_t MakeDshotPacket(uint16_t throttle, bool telemetry);
        void BuildDshotSymbols(uint16_t packet, rmt_symbol_word_t* symbols, uint16_t count);
        //uint8_t ComputeChecksum(uint16_t value);

    private:
        gpio_num_t pin_ = GPIO_NUM_NC;
        rmt_channel_handle_t channel_ = nullptr;
        rmt_encoder_handle_t encoder_ = nullptr;
        bool initialized_ = false;
};