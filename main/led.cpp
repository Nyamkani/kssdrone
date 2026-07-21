#include "led.h"



esp_err_t LedController::Initialize()
{
    //led gpio config part is at the gpio_config.h, so we just need to set the initial state here.

    this->led_onoff_ = false;

    this->state_ = LedState::OFF;

    this->period_ = 0;

    this->error_blink_count_ = 0;

    return ESP_OK;
}


esp_err_t LedController::Run()
{
    LedState state = this->state_;

    switch (state)
    {
        case LedState::OFF:
            this->period_ = 0;
            break;
        case LedState::INIT:
            this->period_ = 100*0.001f; // 10Hz
            break;
        case LedState::IMU_BIAS_CALIBRATING:
            this->period_ = 2000*0.001f; // 0.5Hz
            break;
        case LedState::DISARMED:
            this->period_ = 1000*0.001f; // 1Hz
            break;
        case LedState::ARMING:
            this->period_ = 300*0.001f; // 30Hz
            break;
        case LedState::ARMED:
            this->period_ = 0; 
            break;
        case LedState::ERROR:
            this->period_ = 1000*0.001f; // 1Hz
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t LedController::Update(float dt)
{
    this->CheckStateChange();

    if(this->state_ == LedState::OFF)
    {
        if(this->led_onoff_)
        {
            BoardSetBuzzerLed(false);
            this->led_onoff_ = false;
        }
        return ESP_OK;
    }
    else if(this->state_ == LedState::ERROR)
    {
        bool dt_on_ = false;

        // In error state, we want to blink the LED with a special pattern: 100ms on, 100ms off
        this->led_dt_ += dt;

        if(this->led_dt_ >= this->period_)
        {
            while(this->led_dt_ >= this->period_)
            {
                this->led_dt_ -= this->period_;
            }
            dt_on_ = true;
        }

        if(dt_on_)
        {
            uint16_t err_seq = this->error_blink_count_++;

            if (err_seq < 6)
            {
                this->Toggle(); // 6 blinks with 100ms on, 100ms off
            }
            else if (err_seq < 10)
            {
                this->SetLedOff(); // pause
            }
            else
            {
                this->error_blink_count_ = 0;
            }
        }
        return ESP_OK;
    }
    else if (this->state_ == LedState::ARMED)
    {
        // if (!this->led_onoff_)
        // {
            this->SetLedOn();
            this->led_onoff_ = true;
        // }
    }
    else
    {
        this->led_dt_ += dt;

        if(this->led_dt_ >= this->period_)
        {
            while(this->led_dt_ >= this->period_)
            {
                this->led_dt_ -= this->period_;
            }
            this->Toggle();
        }
    }

    return ESP_OK;
}
         

void LedController::Toggle()
{
    if (this->led_onoff_)
    {
        this->SetLedOff();
        this->led_onoff_ = false;
    }
    else
    {
        this->SetLedOn();
        this->led_onoff_ = true;
    }
 
}

void LedController::SetLedOn()
{
    BoardSetBuzzerLed(false);

    return;
}

void LedController::SetLedOff()
{
    BoardSetBuzzerLed(true);

    return;
}

void LedController::CheckStateChange()
{
    if(this->prev_state != this->state_)
    {
        this->led_dt_ = 0.0f; // Reset timer on state change
        this->SetLedOff();
        this->Run(); // Update period based on new state
        this->prev_state = this->state_;
    }
    return;
}