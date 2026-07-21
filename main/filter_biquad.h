#pragma once

#include <math.h>

class BiquadNotch
{
    public:
        inline void Configure(float sample_hz, float notch_hz, float q)
        {
            const float pi = 3.14159265358979323846f;
            const float w0 = 2.0f * pi * notch_hz / sample_hz;
            const float c = cosf(w0);
            const float s = sinf(w0);
            const float alpha = s / (2.0f * q);

            const float a0 = 1.0f + alpha;

            b0_ = 1.0f / a0;
            b1_ = (-2.0f * c) / a0;
            b2_ = 1.0f / a0;
            a1_ = (-2.0f * c) / a0;
            a2_ = (1.0f - alpha) / a0;
        }

        inline float Update(float x)
        {
            const float y =
                b0_ * x +
                b1_ * x1_ +
                b2_ * x2_ -
                a1_ * y1_ -
                a2_ * y2_;

            x2_ = x1_;
            x1_ = x;

            y2_ = y1_;
            y1_ = y;

            return y;
        }

        inline void Reset()
        {
            x1_ = 0.0f;
            x2_ = 0.0f;
            y1_ = 0.0f;
            y2_ = 0.0f;
        }

    private:
        float b0_ = 1.0f;
        float b1_ = 0.0f;
        float b2_ = 0.0f;
        float a1_ = 0.0f;
        float a2_ = 0.0f;

        float x1_ = 0.0f;
        float x2_ = 0.0f;
        float y1_ = 0.0f;
        float y2_ = 0.0f;
};