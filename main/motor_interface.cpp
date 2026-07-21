
#include "motor_interface.h"

// M1: Front Left
// M2: Front Right
// M3: Rear Right
// M4: Rear Left

//throttle -> Total thrust
//roll -> left/right tilt
//pitch -> forward/backward tilt
//yaw -> rotation around vertical axis

/*
Motor layout (X configuration):

          Front
            ↑
            |

        M1       M2

        M4       M3
*/
/*
M1  CCW  -> real -> m2
M2  CW   -> real -> m4
M3  CCW  -> real -> m1
M4  CW   -> readl -> m3
*/

//ex ) roll -> right tilt -> M1, M4 increase, M2, M3 decrease
//ex ) pitch -> backward tilt -> M1, M2 increase, M3, M4 decrease
//ex ) yaw -> CCW rotation -> M1, M3 increase, M2, M4 decrease

/*
M1 = throttle + roll - pitch + yaw
M2 = throttle - roll - pitch - yaw
M3 = throttle - roll + pitch + yaw
M4 = throttle + roll + pitch - yaw
*/


esp_err_t MotorInterface::Initialize()
{
    esp_err_t ret = this->InitMotors();
    if (ret != ESP_OK) 
    {
        return ret;
    }

    this->state_ = MotorState::RUN;

    this->armed_.store(false);

    return ESP_OK;
}


esp_err_t MotorInterface::InitMotors() 
{
    esp_err_t ret;

    ret = this->motors_[0].Initialize(PIN_ESC1);
    if (ret != ESP_OK) return ret;

    ret = this->motors_[1].Initialize(PIN_ESC2);
    if (ret != ESP_OK) return ret;

    ret = this->motors_[2].Initialize(PIN_ESC3);
    if (ret != ESP_OK) return ret;

    ret = this->motors_[3].Initialize(PIN_ESC4);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}


esp_err_t MotorInterface::SetMotorOutput(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    if (this->state_ != MotorState::RUN)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!this->armed_.load())
    {
        m1 = m2 = m3 = m4 = 0;
    }

    m1 = std::min<uint16_t>(m1, 2047);
    m2 = std::min<uint16_t>(m2, 2047);
    m3 = std::min<uint16_t>(m3, 2047);
    m4 = std::min<uint16_t>(m4, 2047);

    esp_err_t ret;

    // vTaskSuspendAll();
    // 1. send all 
    ret = this->motors_[0].SendThrottle(m1);
    if (ret != ESP_OK) return ret;

    ret = this->motors_[1].SendThrottle(m2);
    if (ret != ESP_OK) return ret;

    ret = this->motors_[2].SendThrottle(m3);
    if (ret != ESP_OK) return ret;

    ret = this->motors_[3].SendThrottle(m4);
    if (ret != ESP_OK) return ret;

    // xTaskResumeAll();

    //2. wait all
    // ret = this->motors_[0].WaitDone(pdMS_TO_TICKS(1));
    // if (ret != ESP_OK) return ret;

    // ret = this->motors_[1].WaitDone(pdMS_TO_TICKS(1));
    // if (ret != ESP_OK) return ret;

    // ret = this->motors_[2].WaitDone(pdMS_TO_TICKS(1));
    // if (ret != ESP_OK) return ret;

    // ret = this->motors_[3].WaitDone(pdMS_TO_TICKS(1));
    // if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t MotorInterface::Arm()
{
    this->armed_.store(true);
    return ESP_OK;
}

esp_err_t MotorInterface::Disarm()
{
    this->armed_.store(false);

    if(this->state_ == MotorState::RUN)
    {
        this->SetMotorOutput(0, 0, 0, 0);
    }

    return ESP_OK;
}



////////////////////////////////////////////////////////////////

MotorInterface::MotorInterface()
{

}

MotorInterface::~MotorInterface()
{
    //this->StopTaskRequest();
}