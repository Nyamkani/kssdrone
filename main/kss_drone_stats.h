#pragma once

#include <cstdint>
#include <climits>
#include <algorithm>

struct LoopTimingStats
{
    float min_dt_s = 999.0f;
    float max_dt_s = 0.0f;
    float sum_dt_s = 0.0f;
    float elapsed_s = 0.0f;

    uint32_t count = 0;
    uint32_t over_2ms = 0;
    uint32_t over_5ms = 0;
    uint32_t over_10ms = 0;
    uint32_t wdt_guard = 0;

    void Update(float raw_dt_s)
    {
        min_dt_s = std::min(min_dt_s, raw_dt_s);
        max_dt_s = std::max(max_dt_s, raw_dt_s);
        sum_dt_s += raw_dt_s;
        elapsed_s += raw_dt_s;
        count++;

        if (raw_dt_s > 0.002f) over_2ms++;
        if (raw_dt_s > 0.005f) over_5ms++;
        if (raw_dt_s > 0.010f) over_10ms++;
    }

    float AvgDtS() const
    {
        return count > 0 ? sum_dt_s / static_cast<float>(count) : 0.0f;
    }

    void Reset()
    {
        min_dt_s = 999.0f;
        max_dt_s = 0.0f;
        sum_dt_s = 0.0f;
        elapsed_s = 0.0f;

        count = 0;
        over_2ms = 0;
        over_5ms = 0;
        over_10ms = 0;
        wdt_guard = 0;
    }
};

struct ImuFrameStats
{
    static constexpr int64_t STALE_LIMIT_US = 2000;

    uint32_t frame_count = 0;
    uint32_t invalid_count = 0;
    uint32_t stale_count = 0;

    uint32_t delta0_count = 0;   // 같은 frame 재사용
    uint32_t delta1_count = 0;   // 정상 다음 frame
    uint32_t delta2_count = 0;   // 1개 건너뜀
    uint32_t delta3p_count = 0;  // 2개 이상 건너뜀

    uint64_t age_sum_us = 0;
    int64_t age_min_us = INT64_MAX;
    int64_t age_max_us = 0;

    uint32_t last_seq = 0;
    uint32_t last_seq_for_log = 0;
    uint32_t last_sample_count = 0;

    void Update(uint32_t seq,
                uint32_t sample_count,
                int64_t timestamp_us,
                int64_t now_us,
                bool valid)
    {
        if (!valid)
        {
            invalid_count++;
            return;
        }

        const int64_t age_us = now_us - timestamp_us;

        frame_count++;

        if (age_us >= 0)
        {
            age_sum_us += static_cast<uint64_t>(age_us);
        }

        age_min_us = std::min(age_min_us, age_us);
        age_max_us = std::max(age_max_us, age_us);

        if (age_us > STALE_LIMIT_US)
        {
            stale_count++;
        }

        if (last_seq != 0)
        {
            const uint32_t delta = seq - last_seq;

            if (delta == 0)
            {
                delta0_count++;
            }
            else if (delta == 1)
            {
                delta1_count++;
            }
            else if (delta == 2)
            {
                delta2_count++;
            }
            else
            {
                delta3p_count++;
            }
        }

        last_seq = seq;
        last_seq_for_log = seq;
        last_sample_count = sample_count;
    }

    float AvgAgeUs() const
    {
        return frame_count > 0
            ? static_cast<float>(age_sum_us) / static_cast<float>(frame_count)
            : 0.0f;
    }

    int64_t MinAgeUsForLog() const
    {
        return age_min_us == INT64_MAX ? 0 : age_min_us;
    }

    void ResetKeepLastSeq()
    {
        frame_count = 0;
        invalid_count = 0;
        stale_count = 0;

        delta0_count = 0;
        delta1_count = 0;
        delta2_count = 0;
        delta3p_count = 0;

        age_sum_us = 0;
        age_min_us = INT64_MAX;
        age_max_us = 0;

        last_seq_for_log = last_seq;
        last_sample_count = 0;
    }
};