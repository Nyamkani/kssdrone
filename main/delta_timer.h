#include "esp_timer.h"


class DeltaTime
{
public:
   inline esp_err_t Initialize()
    {
        Reset();
        return ESP_OK;
    }

    inline void Reset()
    {
        prev_time_us_ = esp_timer_get_time();
        first_ = true;
    }

    inline float Update()
    {
        int64_t now = esp_timer_get_time();

        if (first_)
        {
            prev_time_us_ = now;
            first_ = false;
            return default_dt_;  // 안전한 초기값
        }

        int64_t diff = now - prev_time_us_;
        prev_time_us_ = now;

        float dt = diff * 1e-6f; // us → seconds

        // sanity check
        if (dt <= 0.0f || dt > max_dt_)
        {
            return default_dt_;
        }

        return dt;
    }

private:
    int64_t prev_time_us_ = 0;
    bool first_ = true;

    float default_dt_ = 0.001f; // 1ms (1kHz)
    float max_dt_ = 0.02f;      // 20ms 이상이면 비정상으로 간주
};