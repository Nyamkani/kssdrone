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
    uint32_t armed_overrun_score = 0;
    float armed_guard_elapsed_dt = 0.0f;

#if MAIN_LOOP_RATE_HZ <= 1000
    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t last_wake_time = xTaskGetTickCount();
#else
    float force_idle_elapsed_dt = 0.0f;
    int64_t next_loop_us = esp_timer_get_time();
#endif

    while (true)
    {
        esp_err_t ret = ESP_OK;

        const int64_t loop_start_us = esp_timer_get_time();

        float raw_dt = this->dt_.Update();
        float dt = std::clamp(
            raw_dt,
            MAIN_LOOP_DT_MIN_SEC,
            MAIN_LOOP_DT_MAX_SEC
        );

#if MAIN_LOOP_RATE_HZ > 1000
        next_loop_us += MAIN_LOOP_PERIOD_US;
#endif

        this->UpdateMainLoopStats(raw_dt);
        this->TryPrintMainLoopStats();
        this->TryPrintArmedProfileStats();

        if (this->task_stop_.load())
        {
            break;
        }

        ret = this->FastBackgroundJobs(dt);
        if (ret != ESP_OK)
        {
            this->ChangeState(DroneState::ERR);
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

        this->slow_bg_dt_accum_ += dt;
        if (this->slow_bg_dt_accum_ >= BACKGROUND_SLOW_JOB_TIME)
        {
            ret = this->SlowBackgroundJobs(this->slow_bg_dt_accum_);
            this->slow_bg_dt_accum_ = 0.0f;

            if (ret != ESP_OK)
            {
                this->ChangeState(DroneState::ERR);
            }
        }

        /*
         * WDT / overrun guard
         *
         * 목적:
         * - ARMED 상태에서 deadline을 계속 놓치면 task가 거의 쉬지 못할 수 있음.
         * - 일정 수준 이상 overrun이 누적되면 강제로 1 tick block.
         *
         * 1kHz:
         * - vTaskDelayUntil() 자체가 block을 만들기 때문에 일반적으로 guard가 거의 없어야 함.
         *
         * 2kHz 이상:
         * - busy-wait 기반이므로 guard가 중요함.
         */
        bool forced_idle_delay = false;

        if (this->state_ == DroneState::ARMED)
        {
            armed_guard_elapsed_dt += raw_dt;

            if (raw_dt > MAIN_LOOP_WDT_GUARD_DT_SEC)
            {
                armed_overrun_score++;
            }
            else if (armed_overrun_score > 0)
            {
                armed_overrun_score--;
            }

            const bool score_guard =
                armed_overrun_score >= MAIN_LOOP_WDT_GUARD_SCORE_LIMIT;

            const bool time_guard =
                armed_guard_elapsed_dt >= MAIN_LOOP_WDT_GUARD_TIME_SEC &&
                armed_overrun_score > 0;

            if (score_guard || time_guard)
            {
                this->loop_stats_.wdt_guard++;

                /*
                 * deadline을 계속 놓치는 경우 강제로 block.
                 */
                vTaskDelay(1);

#if MAIN_LOOP_RATE_HZ <= 1000
                /*
                 * tick 기반에서는 vTaskDelay(1) 이후 기준 tick을 다시 잡아야
                 * vTaskDelayUntil()이 과거 deadline을 따라잡으려 하지 않는다.
                 */
                last_wake_time = xTaskGetTickCount();
#else
                /*
                 * us deadline 기반에서도 delay 이후 현재 시간으로 재동기화.
                 */
                next_loop_us = esp_timer_get_time();
#endif

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

#if MAIN_LOOP_RATE_HZ > 1000
        /*
         * 2kHz 이상에서는 busy-wait 때문에 IDLE task가 못 돌 수 있다.
         * 따라서 overrun과 무관하게 주기적으로 1 tick block을 넣는다.
         *
         * 1kHz에서는 vTaskDelayUntil()이 이미 block을 만들기 때문에 사용하지 않는다.
         */
        force_idle_elapsed_dt += dt;

        if (force_idle_elapsed_dt >= MAIN_LOOP_FORCE_IDLE_INTERVAL_SEC)
        {
            this->loop_stats_.wdt_guard++;

            vTaskDelay(1);

            next_loop_us = esp_timer_get_time();

            force_idle_elapsed_dt = 0.0f;
            armed_overrun_score = 0;
            armed_guard_elapsed_dt = 0.0f;

            forced_idle_delay = true;
        }
#endif

#if MAIN_LOOP_RATE_HZ <= 1000
        /*
         * 1kHz 이하:
         * 기존 FreeRTOS tick 기반 periodic loop.
         *
         * 이미 overrun guard로 vTaskDelay(1)을 수행한 loop에서는
         * vTaskDelayUntil()을 추가로 호출하지 않는다.
         */
        if (!forced_idle_delay)
        {
            vTaskDelayUntil(&last_wake_time, period);
        }
#else
        /*
         * 2kHz 이상:
         * FreeRTOS tick으로는 500us/250us 주기를 표현할 수 없으므로
         * esp_timer_get_time() deadline까지 짧게 busy-wait.
         */
        if (!forced_idle_delay)
        {
            const int64_t now_us = esp_timer_get_time();
            int64_t wait_us = next_loop_us - now_us;

            if (wait_us > 0)
            {
                /*
                * 2kHz 기준:
                * 긴 wait는 ROM delay,
                * 마지막 30~50us만 spin.
                */
                if (wait_us > 80)
                {
                    esp_rom_delay_us(static_cast<uint32_t>(wait_us - 40));
                }

                while (esp_timer_get_time() < next_loop_us)
                {
                    __asm__ __volatile__("nop");
                }
            }

            const int64_t now_after_wait_us = esp_timer_get_time();

            if ((now_after_wait_us - next_loop_us) > MAIN_LOOP_PERIOD_US)
            {
                next_loop_us = now_after_wait_us;
            }
        }
#endif

        const int64_t work_time_us = esp_timer_get_time() - loop_start_us;
        (void)work_time_us;
    }
}

