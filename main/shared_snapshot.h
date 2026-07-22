#pragma once

#include <atomic>

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



template <typename T>
class TripleSnapshot
{
    public:
        TripleSnapshot() = default;

        TripleSnapshot(const TripleSnapshot&) = delete;
        TripleSnapshot& operator=(const TripleSnapshot&) = delete;

        void Write(
            const T& data,
            const uint16_t sample_count = 1)
        {
            this->Write(
                data,
                esp_timer_get_time(),
                sample_count);
        }

        void Write(
            const T& data,
            const int64_t timestamp_us,
            const uint16_t sample_count = 1)
        {
            /*
            * write_index_가 가리키는 슬롯은
            * producer만 소유한다.
            */
            SharedSnapshotFrame<T>& frame =
                this->frames_[this->write_index_];

            frame.data = data;
            frame.seq = ++this->seq_;
            frame.timestamp_us = timestamp_us;
            frame.sample_count = sample_count;
            frame.valid = true;

            /*
            * 완성된 write 슬롯을 mailbox에 publish한다.
            *
            * 기존 mailbox 슬롯은 producer가 가져와서
            * 다음 write 슬롯으로 사용한다.
            */
            const uint32_t previous_state =
                this->next_state_.exchange(
                    PackState(this->write_index_, true),
                    std::memory_order_acq_rel);

            this->write_index_ =
                GetIndex(previous_state);
        }

        bool Read(SharedSnapshotFrame<T>& out)
        {
            /*
            * 새로 publish된 frame이 있는지 먼저 확인한다.
            */
            const uint32_t state =
                this->next_state_.load(
                    std::memory_order_acquire);

            if (IsDirty(state))
            {
                /*
                * 기존 read 슬롯을 mailbox에 반환하고,
                * 최신 publish 슬롯의 소유권을 가져온다.
                */
                const uint32_t latest_state =
                    this->next_state_.exchange(
                        PackState(this->read_index_, false),
                        std::memory_order_acq_rel);

                this->read_index_ =
                    GetIndex(latest_state);
            }

            /*
            * read_index_ 슬롯은 consumer가 독점하므로
            * 복사 중 producer가 덮어쓰지 않는다.
            */
            out = this->frames_[this->read_index_];

            /*
            * 기존 SharedSnapshot과 동일한 의미:
            * 새 데이터 여부가 아니라 유효한 snapshot 존재 여부.
            */
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
        static constexpr uint32_t DIRTY_MASK = 0x01u;
        static constexpr uint32_t INDEX_SHIFT = 1u;

        static constexpr uint32_t PackState(
            const uint32_t index,
            const bool dirty)
        {
            return
                (index << INDEX_SHIFT) |
                (dirty ? DIRTY_MASK : 0u);
        }

        static constexpr uint32_t GetIndex(
            const uint32_t state)
        {
            return state >> INDEX_SHIFT;
        }

        static constexpr bool IsDirty(
            const uint32_t state)
        {
            return
                (state & DIRTY_MASK) != 0u;
        }

    private:
        SharedSnapshotFrame<T> frames_[3]{};

        /*
        * 초기 슬롯 소유권
        *
        * Consumer read : 0
        * Producer write: 1
        * Shared mailbox: 2
        */
        uint32_t read_index_ = 0;
        uint32_t write_index_ = 1;

        uint32_t seq_ = 0;

        std::atomic<uint32_t> next_state_{
            PackState(2, false)
        };
};

static_assert(
    std::atomic<uint32_t>::is_always_lock_free,
    "TripleSnapshot requires lock-free 32-bit atomic operations.");