#include "kss_drone.h"

esp_err_t KSSDrone::StartTask()
{
    BaseType_t ret = xTaskCreatePinnedToCore(MainTask, "DroneMainTask", 4096 * 2, this, 6, &(this->handle_), 1);

    if (ret != pdPASS)
    {
        this->handle_ = nullptr;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void KSSDrone::StopTaskRequest()
{
    this->task_stop_.store(true);

    TaskHandle_t handle = this->handle_;
    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}

void KSSDrone::MainTask(void* param)
{
    KSSDrone* this_ = static_cast<KSSDrone*>(param);
    if (this_ == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    this_->MainLoop();
    this_->handle_ = nullptr;
    vTaskDelete(nullptr);
}

void KSSDrone::MainLoop()
{
    const TickType_t period = pdMS_TO_TICKS(1); // 1000 Hz
    TickType_t last_wake_time = xTaskGetTickCount();

    constexpr float WDT_GUARD_DT_SEC = 0.00105f;
    constexpr uint32_t WDT_GUARD_SCORE_LIMIT = 60;
    constexpr float WDT_GUARD_TIME_SEC = 0.500f;


    uint32_t armed_overrun_score = 0;
    float armed_guard_elapsed_dt = 0.0f;

    while (true)
    {
        esp_err_t ret = ESP_OK;

        float raw_dt = this->dt_.Update();
        float dt = std::clamp(raw_dt, 0.0005f, 0.005f);

        this->UpdateMainLoopStats(raw_dt);
        this->TryPrintMainLoopStats();

        if (this->task_stop_.load())
        {
            break;
        }

        switch (this->state_)
        {
            case DroneState::INIT:
                ret = this->Init(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::IMU_BIAS_CALIBRATING:
                ret = this->ImuBiasCalibrating(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::DISARMED:
                ret = this->Disarmed(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::ARMING:
                ret = this->Arming(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::ARMED:
                ret = this->Armed(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::LANDING:
                ret = this->Landing(dt);
                if (ret != ESP_OK)
                {
                    this->ChangeState(DroneState::ERR);
                }
                break;

            case DroneState::ERR:
                this->Error(dt);
                break;

            default:
                break;
        }

        ret = this->FastBackgroundJobs(dt);
        if (ret != ESP_OK)
        {
            this->ChangeState(DroneState::ERR);
        }

        this->slow_bg_dt_accum_ += dt;
        if (this->slow_bg_dt_accum_ >= BACKGROUND_SLOW_JOB_TIME) //100hz
        {
            ret = this->SlowBackgroundJobs(this->slow_bg_dt_accum_);
            this->slow_bg_dt_accum_ = 0.0f;

            if (ret != ESP_OK)
            {
                this->ChangeState(DroneState::ERR);
            }
        }

        /*
        * WDT guard
        *
        * raw_dt는 이번 loop 시작 시점에서 측정한 "직전 loop 주기"이다.
        * ARMED 상태에서 raw_dt가 계속 1ms를 넘는다는 것은
        * DroneMainTask가 deadline을 계속 놓치고 있다는 뜻이다.
        *
        * 이 경우 vTaskDelayUntil()은 거의 sleep하지 못하므로,
        * IDLE1 task가 실행되지 못해서 Task WDT가 발생한다.
        *
        * 따라서 일정 횟수 이상 연속 overrun이면 vTaskDelay(1)을 강제로 넣어
        * DroneMainTask를 blocked 상태로 만들고 IDLE1 실행 기회를 보장한다.
        */
        bool forced_idle_delay = false;

        if (this->state_ == DroneState::ARMED)
        {
            armed_guard_elapsed_dt += raw_dt;

            if (raw_dt > WDT_GUARD_DT_SEC)
            {
                armed_overrun_score++;
            }
            else if (armed_overrun_score > 0)
            {
                armed_overrun_score--;
            }

            const bool score_guard =
                armed_overrun_score >= WDT_GUARD_SCORE_LIMIT;

            const bool time_guard =
                armed_guard_elapsed_dt >= WDT_GUARD_TIME_SEC &&
                armed_overrun_score > 0;

            if (score_guard || time_guard)
            {
                this->loop_stats_.wdt_guard++;

                vTaskDelay(1);

                last_wake_time = xTaskGetTickCount();

                armed_overrun_score = 0;
                armed_guard_elapsed_dt = 0.0f;
                forced_idle_delay = true;
            }
        }
        else
        {
            armed_overrun_score = 0;
            armed_guard_elapsed_dt = 0.0f;
        }

        if (!forced_idle_delay)
        {
            vTaskDelayUntil(&last_wake_time, period);
        }
    }
}

