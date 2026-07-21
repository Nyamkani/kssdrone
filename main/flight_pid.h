#pragma once

#include "pid.h"

#include <algorithm>
#include <cmath>
#include <packets.h>

#include "flight_pid_config.h"

struct AttitudeTarget
{
    float throttle = 0.0f;

    // angle mode target
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    // rate mode target
    float roll_rate_rad_s = 0.0f;
    float pitch_rate_rad_s = 0.0f;
    float yaw_rate_rad_s = 0.0f;

    bool yaw_hold_enable = false;

    DroneMode mode = DroneMode::RATE_ACRO; // 0: rate/acro, 1: angle/self-level
};

struct AttitudeState
{
    float roll_rad = 0.0f;      // estimated roll rad
    float pitch_rad = 0.0f;     // estimated pitch rad
    float yaw_rad = 0.0f;       // estimated yaw rad

    float gyro_x_rad_s = 0.0f;    // current roll rate rad/s
    float gyro_y_rad_s = 0.0f;    // current pitch rate rad/s
    float gyro_z_rad_s = 0.0f;    // current yaw rate rad/s
};

struct FlightPIDOutput
{
    float throttle = 0.0f;

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    float roll_rate_target = 0.0f;
    float pitch_rate_target = 0.0f;
};

class FlightPIDController
{
    public:
        FlightPIDController() = default;

        inline esp_err_t Initialize()
        {
            this->Reset();

            this->SetRollAnglePID(
                PIDGains{ROLL_ANGLE_P, ROLL_ANGLE_I, ROLL_ANGLE_D},
                PIDLimits{-RP_ANGLE_INTEGRAL, RP_ANGLE_INTEGRAL, -ANGLE_PID_MAX_RATE_RAD_S, ANGLE_PID_MAX_RATE_RAD_S}
            );

            this->SetPitchAnglePID(
                PIDGains{PITCH_ANGLE_P, PITCH_ANGLE_I, PITCH_ANGLE_D},
                PIDLimits{-RP_ANGLE_INTEGRAL, RP_ANGLE_INTEGRAL, -ANGLE_PID_MAX_RATE_RAD_S, ANGLE_PID_MAX_RATE_RAD_S}
            );

            this->SetRollRatePID(
                PIDGains{ROLL_RATE_P, ROLL_RATE_I, ROLL_RATE_D},
                PIDLimits{-RATE_INTEGRAL, RATE_INTEGRAL, -RATE_PID_MAX_RATE_RAD, RATE_PID_MAX_RATE_RAD}
            );

            this->SetPitchRatePID(
                PIDGains{PITCH_RATE_P, PITCH_RATE_I, PITCH_RATE_D},
                PIDLimits{-RATE_INTEGRAL, RATE_INTEGRAL, -RATE_PID_MAX_RATE_RAD, RATE_PID_MAX_RATE_RAD}
            );

            this->SetYawRatePID(
                PIDGains{YAW_RATE_P, YAW_RATE_I, YAW_RATE_D},
                PIDLimits{-YAW_RATE_RATE_INTEGRAL, YAW_RATE_RATE_INTEGRAL, -YAW_RATE_PID_MAX_RATE_RAD, YAW_RATE_PID_MAX_RATE_RAD}
            );

            return ESP_OK;
        }


        inline void Reset()
        {
            roll_angle_pid_.Reset();
            pitch_angle_pid_.Reset();

            roll_rate_pid_.Reset();
            pitch_rate_pid_.Reset();
            yaw_rate_pid_.Reset();
        }

        inline void ResetIntegrator()
        {
            roll_angle_pid_.ResetIntegrator();
            pitch_angle_pid_.ResetIntegrator();

            roll_rate_pid_.ResetIntegrator();
            pitch_rate_pid_.ResetIntegrator();
            yaw_rate_pid_.ResetIntegrator();
        }


        inline void SetRollAnglePID(const PIDGains& gains, const PIDLimits& limits)
        {
            roll_angle_pid_.SetGains(gains);
            roll_angle_pid_.SetLimits(limits);
        }

        inline void SetPitchAnglePID(const PIDGains& gains, const PIDLimits& limits)
        {
            pitch_angle_pid_.SetGains(gains);
            pitch_angle_pid_.SetLimits(limits);
        }

        inline void SetRollRatePID(const PIDGains& gains, const PIDLimits& limits)
        {
            roll_rate_pid_.SetGains(gains);
            roll_rate_pid_.SetLimits(limits);
        }

        inline void SetPitchRatePID(const PIDGains& gains, const PIDLimits& limits)
        {
            pitch_rate_pid_.SetGains(gains);
            pitch_rate_pid_.SetLimits(limits);
        }

        inline void SetYawRatePID(const PIDGains& gains, const PIDLimits& limits)
        {
            yaw_rate_pid_.SetGains(gains);
            yaw_rate_pid_.SetLimits(limits);
        }

        inline void SetDerivativeFilterAlpha(float alpha)
        {
            roll_angle_pid_.SetDerivativeFilterAlpha(alpha);
            pitch_angle_pid_.SetDerivativeFilterAlpha(alpha);

            roll_rate_pid_.SetDerivativeFilterAlpha(alpha);
            pitch_rate_pid_.SetDerivativeFilterAlpha(alpha);
            yaw_rate_pid_.SetDerivativeFilterAlpha(alpha);
        }

        inline FlightPIDOutput Update(const AttitudeTarget& target,
                                const AttitudeState& state,
                                float dt,
                                bool output_saturated = false)
        {
            FlightPIDOutput out{};
            out.throttle = target.throttle;

            if (target.mode == DroneMode::RATE_ACRO) 
            {
                out.roll_rate_target = target.roll_rate_rad_s;
                out.pitch_rate_target = target.pitch_rate_rad_s;
            }
            else
            {
                out.roll_rate_target = roll_angle_pid_.Update(
                    target.roll_rad,
                    state.roll_rad,
                    dt,
                    false
                );

                out.pitch_rate_target = pitch_angle_pid_.Update(
                    target.pitch_rad,
                    state.pitch_rad,
                    dt,
                    false
                );
            }

            out.roll = roll_rate_pid_.Update(
                out.roll_rate_target,
                state.gyro_x_rad_s,
                dt,
                output_saturated
            );

            out.pitch = pitch_rate_pid_.Update(
                out.pitch_rate_target,
                state.gyro_y_rad_s,
                dt,
                output_saturated
            );

            float yaw_rate_target = target.yaw_rate_rad_s;
            float yaw_error = 0.0f;
            float yaw_hold_rate = 0.0f;

            if (target.yaw_hold_enable)
            {
                yaw_error = WrapPI(target.yaw_rad - state.yaw_rad);

                yaw_hold_rate = std::clamp(
                    yaw_error * YAW_ANGLE_P,
                    -MAX_YAW_HOLD_RATE_RAD_S,
                    MAX_YAW_HOLD_RATE_RAD_S
                );

                yaw_rate_target += yaw_hold_rate;
            }

            // float yaw_rate_target = target.yaw_rate_rad_s;
            // float yaw_error = 0.0f;
            // float yaw_hold_rate = 0.0f;

            // if (target.yaw_hold_enable)
            // {
            //     yaw_error = WrapPI(target.yaw_rad - state.yaw_rad);

            //     yaw_hold_rate = yaw_error * YAW_ANGLE_P;

            //     const float yaw_error_deadband = DEG2RAD(2.0f);
            //     const float min_yaw_hold_rate  = DEG2RAD(00.0f);

            //     if (std::fabs(yaw_error) > yaw_error_deadband)
            //     {
            //         if (yaw_hold_rate > 0.0f)
            //         {
            //             yaw_hold_rate = std::max(yaw_hold_rate, min_yaw_hold_rate);
            //         }
            //         else
            //         {
            //             yaw_hold_rate = std::min(yaw_hold_rate, -min_yaw_hold_rate);
            //         }
            //     }

            //     yaw_hold_rate = std::clamp(
            //         yaw_hold_rate,
            //         -MAX_YAW_HOLD_RATE_RAD_S,
            //         MAX_YAW_HOLD_RATE_RAD_S
            //     );

            //     yaw_rate_target += yaw_hold_rate;
            // }

            this->last_yaw_error_ = yaw_error;
            this->last_yaw_hold_rate_ = yaw_hold_rate;
            this->last_yaw_rate_target_ = yaw_rate_target;

            out.yaw = yaw_rate_pid_.Update(
                yaw_rate_target,
                state.gyro_z_rad_s,
                dt,
                output_saturated
            );

            return out;
        }

        inline float GetRollAngleIntegralRaw() const {return roll_angle_pid_.GetIntegral();}
        inline float GetPitchAngleIntegralRaw() const {return pitch_angle_pid_.GetIntegral();}

        inline float GetRollRatePterm() const {return roll_rate_pid_.GetPTerm();}
        inline float GetRollRateIterm() const {return roll_rate_pid_.GetITerm();}
        inline float GetRollRateDterm() const {return roll_rate_pid_.GetDTerm();}

        inline float GetPitchRatePterm() const {return pitch_rate_pid_.GetPTerm();}
        inline float GetPitchRateIterm() const {return pitch_rate_pid_.GetITerm();}
        inline float GetPitchRateDterm() const {return pitch_rate_pid_.GetDTerm();}

        inline float GetYawRatePterm() const {return yaw_rate_pid_.GetPTerm();}
        inline float GetYawRateIterm() const {return yaw_rate_pid_.GetITerm();}
        inline float GetYawRateDterm() const {return yaw_rate_pid_.GetDTerm();}

        inline float GetRollAnglePterm() const { return roll_angle_pid_.GetPTerm(); }
        inline float GetRollAngleITerm() const { return roll_angle_pid_.GetITerm(); }
        inline float GetRollAngleDterm() const { return roll_angle_pid_.GetDTerm(); }

        inline float GetPitchAnglePterm() const { return pitch_angle_pid_.GetPTerm(); }
        inline float GetPitchAngleITerm() const { return pitch_angle_pid_.GetITerm(); }
        inline float GetPitchAngleDterm() const { return pitch_angle_pid_.GetDTerm(); }

        inline float GetRollRateLastOutput() const { return roll_rate_pid_.GetLastOutput(); }
        inline float GetPitchRateLastOutput() const { return pitch_rate_pid_.GetLastOutput(); }
        inline float GetYawRateLastOutput() const { return yaw_rate_pid_.GetLastOutput(); }

        inline float GetRollAngleLastOutput() const { return roll_angle_pid_.GetLastOutput(); }
        inline float GetPitchAngleLastOutput() const { return pitch_angle_pid_.GetLastOutput(); }

        inline float GetRollRateLastError() const { return roll_rate_pid_.GetLastError(); }
        inline float GetPitchRateLastError() const { return pitch_rate_pid_.GetLastError(); }
        inline float GetYawRateLastError() const { return yaw_rate_pid_.GetLastError(); }

        inline float GetRollAngleLastError() const { return roll_angle_pid_.GetLastError(); }
        inline float GetPitchAngleLastError() const { return pitch_angle_pid_.GetLastError(); }

        inline float GetLastYawError() const { return last_yaw_error_; }
        inline float GetLastYawHoldRate() const { return last_yaw_hold_rate_; }
        inline float GetLastYawRateTarget() const { return last_yaw_rate_target_; }

        inline float WrapPI(float x)
        {
            while (x > M_PI)  x -= 2.0f * M_PI;
            while (x < -M_PI) x += 2.0f * M_PI;
            return x;
        }

    private:
        // 1st stage
        PIDController2 roll_angle_pid_;
        PIDController2 pitch_angle_pid_;

        // 2nd stage
        PIDController2 roll_rate_pid_;
        PIDController2 pitch_rate_pid_;
        PIDController2 yaw_rate_pid_;

        float last_yaw_error_ = 0.0f;
        float last_yaw_hold_rate_ = 0.0f;
        float last_yaw_rate_target_ = 0.0f;

};