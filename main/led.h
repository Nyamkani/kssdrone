#pragma once

#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "gpio_config.h"

enum class LedState
{
    OFF,
    INIT,
    IMU_BIAS_CALIBRATING,
    DISARMED,
    ARMING,
    ARMED,
    FAILSAFE_LANDING,
    ERROR
};

class LedController
{
    public:
        LedController() = default;
        virtual ~LedController() = default;

        esp_err_t Initialize();
        esp_err_t Run();
        esp_err_t Update(float dt);
        void CheckStateChange();
        void Toggle();

        inline void SetLedOffMode()
        {
            if(this->state_ != LedState::OFF)
            {
                this->state_ = LedState::OFF;
            }
        }
        inline void SetLedInitMode()
        {
            if(this->state_ != LedState::INIT)
            {
                this->state_ = LedState::INIT;
            }
        }
        inline void SetLedCalibratingMode()
        {
            if(this->state_ != LedState::IMU_BIAS_CALIBRATING)
            {
                this->state_ = LedState::IMU_BIAS_CALIBRATING;
            }
        }

        inline void SetLedDisarmedMode()
        {
            if(this->state_ != LedState::DISARMED)
            {
                this->state_ = LedState::DISARMED;
            }
        }
        inline void SetLedArmingMode()
        {
            if(this->state_ != LedState::ARMING)
            {
                this->state_ = LedState::ARMING;
            }   
        }
        inline void SetLedArmedMode()
        {
            if(this->state_ != LedState::ARMED)
            {
                this->state_ = LedState::ARMED;
            }   
        }
        inline void SetLedFailSafeLandingMode()
        {
            if(this->state_ != LedState::FAILSAFE_LANDING)
            {
                this->state_ = LedState::FAILSAFE_LANDING;
            }   
        }

        inline void SetLedErrorMode()
        {
            if(this->state_ != LedState::ERROR)
            {
                this->state_ = LedState::ERROR;
            }
        }

        void SetLedOn();
        void SetLedOff();

    private:
        LedState prev_state = LedState::OFF;
        LedState state_ = LedState::OFF;
        float period_ = 0.0f; //seconds
        bool led_onoff_ = false;
        float led_dt_ = 0.0f;

        uint16_t error_blink_count_ = 0;
  
};