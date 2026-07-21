#pragma once

#include <cstdint>

/* ESP-IDF */
#include "esp_err.h"

/* Project */
#include "gpio_config.h"

#define BATTERY_MAX_VOLTAGE 8.4f

enum class BatteryState
{
    NORMAL = 0,
    WARNING = 1,
    CRITICAL = 2
};

class BatteryMonitor
{
public:
    BatteryMonitor() = default;
    ~BatteryMonitor() = default;

    esp_err_t Initialize();
    // esp_err_t Update();
    esp_err_t Update(float dt);

    float GetVoltage() const { return voltage_filtered_; }
    float GetPercent() const { return (voltage_filtered_ / BATTERY_MAX_VOLTAGE) * 100.0f; }
    float GetRawVoltage() const { return voltage_raw_; }
    BatteryState GetState() const { return state_; }

    bool IsWarning() const { return state_ == BatteryState::WARNING; }
    bool IsCritical() const { return state_ == BatteryState::CRITICAL; }


private:
    float ConvertRawToBatteryVoltage(int raw) const;
    void UpdateState(float voltage, float dt);

private:
    BatteryState state_ = BatteryState::NORMAL;

    // ADC / divider settings
    float adc_ref_voltage_ = 3.3f;
    float adc_max_count_ = 4095.0f;
    float divider_ratio_ = 6.0f;   // (100k + 20k) / 20k = 6.0

    // Battery thresholds for 2S LiHV
    // float warning_voltage_ = 7.6f;
    // float critical_voltage_ = 7.4f;

    // Battery thresholds for 2S LiPO
    float warning_voltage_ = 7.4f;
    float critical_voltage_ = 6.6f;
    float shutdown_voltage_ = 6.3f;

    // Simple LPF
    float voltage_filtered_ = 0.0f;
    bool first_update_ = true;
    float lpf_alpha_ = 0.70f; // bigger = smoother

    //time based state hold
    float voltage_raw_ = 0.0f;

    float warning_dt_ = 0.0f;
    float critical_dt_ = 0.0f;

    float warning_hold_time_ = 1.0f;
    float critical_hold_time_ = 1.0f;

};