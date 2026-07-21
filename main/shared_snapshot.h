#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"

template <typename T>
struct SharedSnapshotFrame
{
    T data{};

    uint32_t seq = 0;
    int64_t timestamp_us = 0;
    uint16_t sample_count = 1;

    bool valid = false;
};

template <typename T>
class SharedSnapshot
{
public:
    SharedSnapshot() = default;

    SharedSnapshot(const SharedSnapshot&) = delete;
    SharedSnapshot& operator=(const SharedSnapshot&) = delete;

    void Write(const T& data, uint16_t sample_count = 1)
    {
        this->Write(data, esp_timer_get_time(), sample_count);
    }

    void Write(
        const T& data,
        const int64_t timestamp_us,
        const uint16_t sample_count = 1)
    {
        SharedSnapshotFrame<T> frame{};

        frame.data = data;
        frame.timestamp_us = timestamp_us;
        frame.sample_count = sample_count;
        frame.valid = true;

        portENTER_CRITICAL(&this->mux_);

        frame.seq = ++this->seq_;
        this->frame_ = frame;

        portEXIT_CRITICAL(&this->mux_);
    }

    bool Read(SharedSnapshotFrame<T>& out)
    {
        portENTER_CRITICAL(&this->mux_);

        out = this->frame_;

        portEXIT_CRITICAL(&this->mux_);

        return out.valid;
    }

    bool ReadData(T& out)
    {
        SharedSnapshotFrame<T> frame{};

        if (!this->Read(frame))
        {
            return false;
        }

        out = frame.data;
        return true;
    }

private:
    SharedSnapshotFrame<T> frame_{};
    uint32_t seq_ = 0;

    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};