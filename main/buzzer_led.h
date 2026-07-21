#pragma once

#include "driver/gpio.h"


class BuzzerLed
{
    public:
        void Initialize(gpio_num_t pin)
        {
            pin_ = pin;

            gpio_config_t io_conf = {};
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pin_bit_mask = (1ULL << pin_);
            gpio_config(&io_conf);
        }

        void On()
        {
            gpio_set_level(pin_, 1);
        }

        void Off()
        {
            gpio_set_level(pin_, 0);
        }

        void Toggle()
        {
            state_ = !state_;
            gpio_set_level(pin_, state_);
        }

    private:
        gpio_num_t pin_;
        bool state_ = false;
};