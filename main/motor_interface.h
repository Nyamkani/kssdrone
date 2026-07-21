#pragma once

#include <atomic>
#include <cstdint>
#include <algorithm>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "motor.h"
#include "pins.h"

enum class MotorPosition
{
    FrontLeft,
    FrontRight,
    RearRight,
    RearLeft
};

enum class MotorState
{
    INIT = 0,
    IDLE = 1,
    RUN = 2,
    ERR= 10,
};

class MotorInterface
{
    public:
        MotorInterface();
        ~MotorInterface();

        esp_err_t Initialize();
        // void StartTask();
        // void StopTaskRequest();

        esp_err_t SetMotorOutput(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
        esp_err_t Arm();
        esp_err_t Disarm();

    private:
        // static void MainTask(void* param);
        // void MainLoop();

        esp_err_t InitMotors();
        // esp_err_t Run();
        // esp_err_t Error();

    private:
        TaskHandle_t handle_ = nullptr;
        std::atomic<bool> task_stop_{false};
        std::atomic<bool> armed_{false};

        MotorState state_ = MotorState::INIT;

        Motor motors_[4];

        uint16_t target_m1_ = 0;
        uint16_t target_m2_ = 0;
        uint16_t target_m3_ = 0;
        uint16_t target_m4_ = 0;
};