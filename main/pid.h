#pragma once

#include <algorithm>
#include <cmath>

struct PIDGains
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct PIDLimits
{
    float integral_min = -1.0f;
    float integral_max =  1.0f;

    float output_min   = -1.0f;
    float output_max   =  1.0f;
};

class PIDController
{
    public:
        PIDController() = default;

        PIDController(const PIDGains& gains, const PIDLimits& limits)
            : gains_(gains), limits_(limits)
        {
        }

        inline void SetGains(const PIDGains& gains)
        {
            this->gains_ = gains;
        }

        inline void SetLimits(const PIDLimits& limits)
        {
            this->limits_ = limits;
        }

        inline void Reset()
        {
            this->integral_ = 0.0f;
            this->last_error_ = 0.0f;
            this->derivative_filtered_ = 0.0f;
            this->prev_current_ = 0.0f;
            this->first_update_ = true;
        }

        inline void ResetIntegrator()
        {
            this->integral_ = 0.0f;
        }


        inline float Update(float target, float current, float dt, bool output_saturated = false)
        {
            if (dt <= 0.0f)
            {
                return 0.0f;
            }

            const float error = target - current;
            float now_current_ = current;

            // P
            const float p_term = this->gains_.kp * error;

            // I (anti-windup)
            if (!output_saturated)
            {
                this->integral_ += error * dt;
                this->integral_ = std::clamp(this->integral_, this->limits_.integral_min, this->limits_.integral_max);
            }
            const float i_term = this->gains_.ki * this->integral_;

            // D
            float derivative = 0.0f;
            if (!this->first_update_)
            {
                // derivative = (error - this->last_error_) / dt;
                derivative = -(now_current_ - this->prev_current_) /dt;
            }
            else
            {
                this->first_update_ = false;
            }

            // 1st low-pass filter
            this->derivative_filtered_ =
                this->d_filter_alpha_ * this->derivative_filtered_ +
                (1.0f - this->d_filter_alpha_) * derivative;

            const float d_term = this->gains_.kd * this->derivative_filtered_;

            // this->last_error_ = error;
            this->prev_current_ = now_current_;

            float output = p_term + i_term + d_term;
            output = std::clamp(output, this->limits_.output_min, this->limits_.output_max);

            return output;
        }

        inline void SetDerivativeFilterAlpha(float alpha)
        {
            this->d_filter_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
        }

        inline float GetIntegral() const
        {
            return this->integral_;
        }

        inline float GetLastError() const
        {
            return this->last_error_;
        }

    private:
        PIDGains gains_{};
        PIDLimits limits_{};

        float integral_ = 0.0f;
        float last_error_ = 0.0f;
        float derivative_filtered_ = 0.0f;

        float d_filter_alpha_ = 0.7f;
        bool first_update_ = true;
        float prev_current_ = 0.0f;
};


class PIDController2
{
    public:
        PIDController2() = default;

        PIDController2(const PIDGains& gains, const PIDLimits& limits)
            : gains_(gains), limits_(limits)
        {
        }

        inline void SetGains(const PIDGains& gains)
        {
            gains_ = gains;
        }

        inline void SetLimits(const PIDLimits& limits)
        {
            limits_ = limits;
        }

        inline void SetDerivativeFilterAlpha(float alpha)
        {
            d_filter_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
        }

        inline void SetBackCalculationGain(float kb)
        {
            this->backcalc_gain_ = std::max(0.0f, kb);
        }
                
        inline void Reset()
        {
            this->integral_ = 0.0f; 
            this->last_error_ = 0.0f;
            this->derivative_filtered_ = 0.0f;
            this->first_update_ = true;
        }

       inline void ResetIntegrator()
        {
            this->integral_ = 0.0f;
        }


        inline float Update(float target, float current, float dt, bool output_saturated = false)
        {
            if (dt <= 0.0f)
            {
                return 0.0f;
            }

            const float error = target - current;

            // P
            this->p_term_ = gains_.kp * error;

            // D
            float derivative_raw = 0.0f;
            if (!this->first_update_)
            {
                derivative_raw = (error - this->last_error_) / dt;
            }
            else
            {
                this->first_update_ = false;
            }

            this->derivative_filtered_ =
                this->d_filter_alpha_ * this->derivative_filtered_ +
                (1.0f - this->d_filter_alpha_) * derivative_raw;

            this->d_term_ = gains_.kd * this->derivative_filtered_;

            // Current I term
            const float i_term_before = gains_.ki * this->integral_;
            const float u_unsat = this->p_term_ + i_term_before + this->d_term_;

            // 기본 적분 속도
            float i_scale = 1.0f;

            // 큰 오차에서는 I를 천천히
            constexpr float I_ZONE = 0.25f;
            if (std::fabs(error) > I_ZONE)
            {
                i_scale *= 0.20f;
            }

            // 출력 limit 근처에서는 I를 천천히
            constexpr float LIMIT_MARGIN_RATIO = 0.10f;
            const float output_range = limits_.output_max - limits_.output_min;
            const float limit_margin = output_range * LIMIT_MARGIN_RATIO;

            const bool near_limit =
                (u_unsat > limits_.output_max - limit_margin) ||
                (u_unsat < limits_.output_min + limit_margin);

            if (near_limit || output_saturated)
            {
                i_scale *= 0.10f;
            }

            // error와 integral 방향이 반대면, I를 빨리 풀어줌
            const bool i_opposes_error =
                (this->integral_ * error) < 0.0f;

            if (i_opposes_error)
            {
                i_scale = 1.5f;
            }

            // 적분
            this->integral_ += error * dt * i_scale;

            // 항상 약하게 leak
            constexpr float I_LEAK_PER_SEC = 0.3f;
            this->integral_ *= std::exp(-I_LEAK_PER_SEC * dt);

            this->integral_ = std::clamp(
                this->integral_,
                this->limits_.integral_min,
                this->limits_.integral_max
            );

            this->i_term_= gains_.ki * this->integral_;

            this->last_error_ = error;

            float output = this->p_term_ + this->i_term_ + this->d_term_;
            output = std::clamp(output, this->limits_.output_min, this->limits_.output_max);

            // this->last_error_ = error;
            this->last_output_ = output;

            return output;
        }


        // inline float Update(float target, float current, float dt, bool output_saturated = false)
        // {
        //     if (dt <= 0.0f)
        //     {
        //         return 0.0f;
        //     }

        //     const float error = target - current;

        //     // P
        //     const float p_term = gains_.kp * error;

        //     // D
        //     float derivative_raw = 0.0f;
        //     if (!this->first_update_)
        //     {
        //         derivative_raw = (error - last_error_) / dt;
        //     }
        //     else
        //     {
        //         this->first_update_ = false;
        //     }

        //     this->derivative_filtered_ =
        //         this->d_filter_alpha_ * this->derivative_filtered_ +
        //         (1.0f - this->d_filter_alpha_) * derivative_raw;

        //     const float d_term = gains_.kd * this->derivative_filtered_;

        //     // I (before update)
        //     float i_term = gains_.ki * this->integral_;

        //     // Unsaturated output
        //     const float u_unsat = p_term + i_term + d_term;

        //     // Saturated output
        //     const float u_sat = std::clamp(u_unsat, limits_.output_min, limits_.output_max);

        //     // Back-calculation anti-windup
        //     this->integral_ += (error + this->backcalc_gain_ * (u_sat - u_unsat)) * dt;
        //     this->integral_ = std::clamp(this->integral_, limits_.integral_min, limits_.integral_max);

        //     // Recompute I after integration
        //     i_term = gains_.ki * this->integral_;

        //     this->last_error_ = error;

        //     // Final output
        //     float output = p_term + i_term + d_term;
        //     output = std::clamp(output, limits_.output_min, limits_.output_max);

        //     return output;
        // }

        inline float GetIntegral() const
        {
            return integral_;
        }

        inline float GetLastError() const
        {
            return last_error_;
        }

        inline float GetFilteredDerivative() const
        {
            return derivative_filtered_;
        }

        inline PIDGains GetGains() const
        {
            return gains_;
        }

        inline PIDLimits GetLimits() const
        {
            return limits_;
        }

        inline float GetPTerm() const { return p_term_; }
        inline float GetITerm() const { return i_term_; }
        inline float GetDTerm() const { return d_term_; }

        inline float GetLastOutput() const { return last_output_; }

    private:
        PIDGains gains_{};
        PIDLimits limits_{};

        float integral_ = 0.0f;
        float last_error_ = 0.0f;
        float derivative_filtered_ = 0.0f;
        float backcalc_gain_ = 1.0f; // For potential future use in back-calculation anti-windup

        float p_term_ = 0.0f;
        float i_term_ = 0.0f;
        float d_term_ = 0.0f;

        float last_output_ = 0.0f;

        float d_filter_alpha_ = 0.7f;
        bool first_update_ = true;
};

