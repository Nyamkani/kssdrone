#include "adc_battery.h"

#include <algorithm>

#include "esp_log.h"

static const char *tag = "battery_monitor";

esp_err_t BatteryMonitor::Initialize()
{
    state_ = BatteryState::NORMAL;
    voltage_filtered_ = 0.0f;
    first_update_ = true;

    return ESP_OK;
}

esp_err_t BatteryMonitor::Update(float dt)
{
    constexpr int sample_count = 8;

    int raw_sum = 0;

    for (int i = 0; i < sample_count; ++i)
    {
        int raw = 0;
        esp_err_t ret = BoardReadBatteryADCRaw(&raw);
        if (ret != ESP_OK)
        {
            ESP_LOGE(tag, "BoardReadBatteryADCRaw failed");
            return ret;
        }

        raw_sum += raw;
    }

    const int raw_avg = raw_sum / sample_count;
    const float voltage_raw = ConvertRawToBatteryVoltage(raw_avg);

    if (first_update_)
    {
        voltage_filtered_ = voltage_raw;
        first_update_ = false;
    }
    else
    {
        voltage_filtered_ =
            lpf_alpha_ * voltage_filtered_ +
            (1.0f - lpf_alpha_) * voltage_raw;
    }

    if(shutdown_voltage_ < voltage_raw)
    {
        UpdateState(voltage_filtered_, dt);
    }
    else
    {
        state_ = BatteryState::CRITICAL;
    }

    return ESP_OK;
}

float BatteryMonitor::ConvertRawToBatteryVoltage(int raw) const
{
    const float adc_voltage =
        (static_cast<float>(raw) / adc_max_count_) * adc_ref_voltage_;

    const float battery_voltage = adc_voltage * divider_ratio_;
    return battery_voltage;
}

void BatteryMonitor::UpdateState(float voltage, float dt)
{
    if (voltage < critical_voltage_)
        critical_dt_ += dt;
    else
        critical_dt_ = 0.0f;

    if (voltage < warning_voltage_)
        warning_dt_ += dt;
    else
        warning_dt_ = 0.0f;

    if (critical_dt_ >= critical_hold_time_)
        state_ = BatteryState::CRITICAL;
    else if (warning_dt_ >= warning_hold_time_)
        state_ = BatteryState::WARNING;
    else
        state_ = BatteryState::NORMAL;
}