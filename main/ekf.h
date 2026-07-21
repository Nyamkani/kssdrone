#pragma once

#include <cmath>
#include <algorithm>

#define USE_EIGEN_LIBRARY 0

#if USE_EIGEN_LIBRARY

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

struct EKFAttitudeInput
{
    // Accelerometer in g
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 1.0f;

    // Gyroscope in rad/s
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    // delta time in seconds
    float dt = 0.001f;
};

struct EKFAttitudeOutput
{
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

    float roll_rad  = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad   = 0.0f;

    float bgx = 0.0f;
    float bgy = 0.0f;
    float bgz = 0.0f;
};

class QuaternionEKF
{
public:
    using Vec3 = Eigen::Vector3f;
    using Mat7 = Eigen::Matrix<float, 7, 7>;
    using Mat3 = Eigen::Matrix<float, 3, 3>;
    using Mat73 = Eigen::Matrix<float, 7, 3>;
    using Vec7 = Eigen::Matrix<float, 7, 1>;
    using Mat37 = Eigen::Matrix<float, 3, 7>;

public:
    QuaternionEKF()
    {
        Reset();
    }

    void Reset()
    {
        q_ = Eigen::Quaternionf::Identity();
        gyro_bias_.setZero();

        P_.setZero();
        P_.diagonal().setConstant(1e-3f);
    }

    void SetProcessNoise(float q_attitude, float q_bias)
    {
        q_attitude_ = q_attitude;
        q_bias_ = q_bias;
    }

    void SetMeasurementNoise(float r_accel)
    {
        r_accel_ = r_accel;
    }

    void Predict(const EKFAttitudeInput& in)
    {
        const float dt = in.dt;
        if (dt <= 0.0f)
        {
            return;
        }

        const Vec3 gyro(in.gx, in.gy, in.gz);
        const Vec3 omega = gyro - gyro_bias_;

        // q_dot = 0.5 * Omega(w) * q
        Eigen::Quaternionf q_dot;
        q_dot.w() = -0.5f * (omega.x() * q_.x()
                            + omega.y() * q_.y()
                            + omega.z() * q_.z());

        q_dot.x() =  0.5f * (omega.x() * q_.w()
                            + omega.z() * q_.y()
                            - omega.y() * q_.z());

        q_dot.y() =  0.5f * (omega.y() * q_.w()
                            - omega.z() * q_.x()
                            + omega.x() * q_.z());

        q_dot.z() =  0.5f * (omega.z() * q_.w()
                            + omega.y() * q_.x()
                            - omega.x() * q_.y());

        q_.w() += q_dot.w() * dt;
        q_.x() += q_dot.x() * dt;
        q_.y() += q_dot.y() * dt;
        q_.z() += q_dot.z() * dt;

        q_.normalize();

        Mat7 F = Mat7::Identity();

        Eigen::Matrix<float, 4, 3> G;

        const float qw = q_.w();
        const float qx = q_.x();
        const float qy = q_.y();
        const float qz = q_.z();

        G << -qx, -qy, -qz,
            qw, -qz,  qy,
            qz,  qw, -qx,
            -qy,  qx,  qw;

        // q depends on gyro bias with negative sign
        F.block<4,3>(0,4) = -0.5f * G * dt;

        Mat7 Q = Mat7::Zero();
        Q.block<4,4>(0,0).diagonal().setConstant(q_attitude_ * dt);
        Q.block<3,3>(4,4).diagonal().setConstant(q_bias_ * dt);

        P_ = F * P_ * F.transpose() + Q;
        P_ = 0.5f * (P_ + P_.transpose());


        // // 기존 코드와 동일한 단순 공분산 증가
        // for (int i = 0; i < 4; ++i)
        // {
        //     P_(i, i) += q_attitude_ * dt;
        // }

        // for (int i = 4; i < 7; ++i)
        // {
        //     P_(i, i) += q_bias_ * dt;
        // }
    }

    // void Update(const EKFAttitudeInput& in)
    // {
    //     Vec3 a_meas(in.ax, in.ay, in.az);
    //     a_meas = NormalizeVector(a_meas);

    //     Vec3 g_pred = PredictGravityBody();

    //     Vec3 r = a_meas - g_pred;

    //     // 기존 코드와 동일
    //     // Vec3 corr = Cross(g_pred, a_meas);
    //     Vec3 corr = a_meas.cross(g_pred);

    //     const float k = 1.0f / (1.0f + r_accel_);

    //     Eigen::Quaternionf dq;
    //     dq.w() = 1.0f;
    //     dq.x() = 0.5f * corr.x() * k;
    //     dq.y() = 0.5f * corr.y() * k;
    //     dq.z() = 0.5f * corr.z() * k;

    //     q_ = q_ * dq;
    //     q_.normalize();

    //     // 기존 코드와 동일한 bias correction
    //     gyro_bias_.x() += corr.x() * bias_gain_;
    //     gyro_bias_.y() += corr.y() * bias_gain_;
    //     // gyro_bias_.z() += corr.z() * bias_gain_;

    //     // 기존 코드와 동일한 단순 공분산 감소
    //     P_ *= 0.999f;
    // }

    void Update(const EKFAttitudeInput& in)
    {
        Vec3 a_meas(in.ax, in.ay, in.az);
        a_meas = NormalizeVector(a_meas);

        Vec3 g_pred = PredictGravityBody();

        Vec3 r = a_meas - g_pred; //direction of graivity in sensor frame

        // S = HPHᵀ + R
        // K = PHᵀS⁻¹
        // x = x + Kr
        // P = (I-KH)P

        //compute Jacobian H
        Mat37 H;
        H.setZero();

        const float qw = q_.w();
        const float qx = q_.x();
        const float qy = q_.y();
        const float qz = q_.z();

        // gx = 2(qx qz - qw qy)
        H(0, 0) = -2.0f * qy;  // ∂gx/∂qw
        H(0, 1) =  2.0f * qz;  // ∂gx/∂qx
        H(0, 2) = -2.0f * qw;  // ∂gx/∂qy
        H(0, 3) =  2.0f * qx;  // ∂gx/∂qz

        // gy = 2(qw qx + qy qz)
        H(1, 0) =  2.0f * qx;  // ∂gy/∂qw
        H(1, 1) =  2.0f * qw;  // ∂gy/∂qx
        H(1, 2) =  2.0f * qz;  // ∂gy/∂qy
        H(1, 3) =  2.0f * qy;  // ∂gy/∂qz

        // gz = qw² - qx² - qy² + qz²
        H(2, 0) =  2.0f * qw;  // ∂gz/∂qw
        H(2, 1) = -2.0f * qx;  // ∂gz/∂qx
        H(2, 2) = -2.0f * qy;  // ∂gz/∂qy
        H(2, 3) =  2.0f * qz;  // ∂gz/∂qz

        // bgx,bgy,bgz 열은 0


        // S = HPHᵀ + R
        Mat3 R = Mat3::Identity() * r_accel_;
        Mat3 S = H * P_ * H.transpose() + R;

        // K = PHᵀS⁻¹
        Mat73 PHt = P_ * H.transpose();
        //instead of inverse() for better numerical stability
        Mat73 K = PHt * S.ldlt().solve(Mat3::Identity()); 

        // x = x + Kr
        Vec7 dx = K * r;

        //adapt state
        q_.w() += dx(0);
        q_.x() += dx(1);
        q_.y() += dx(2);
        q_.z() += dx(3);
        q_.normalize();

        gyro_bias_.x() += dx(4);
        gyro_bias_.y() += dx(5);
        // gyro_bias_.z() += dx(6);

        // P = (I-KH)P
        Mat7 I = Mat7::Identity();
        Mat7 IKH = I - K * H;

        //joseph form for better numerical stability
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        P_ = 0.5f * (P_ + P_.transpose());

    }

    EKFAttitudeOutput GetOutput() const
    {
        EKFAttitudeOutput out{};
        out.q = q_;
        out.bgx = gyro_bias_.x();
        out.bgy = gyro_bias_.y();
        out.bgz = gyro_bias_.z();

        QuaternionToEuler(q_, out.roll_rad, out.pitch_rad, out.yaw_rad);
        return out;
    }

    Eigen::Quaternionf GetQuaternion() const
    {
        return q_;
    }

    Vec3 GetGyroBias() const
    {
        return gyro_bias_;
    }

    const Mat7& GetCovariance() const
    {
        return P_;
    }

private:
    static Vec3 NormalizeVector(const Vec3& v)
    {
        const float n = v.norm();
        if (n <= 1e-6f)
        {
            return Vec3(0.0f, 0.0f, 1.0f);
        }

        return v / n;
    }

    Vec3 PredictGravityBody() const
    {
        const float qw = q_.w();
        const float qx = q_.x();
        const float qy = q_.y();
        const float qz = q_.z();

        Vec3 g;
        g.x() = 2.0f * (qx * qz - qw * qy);
        g.y() = 2.0f * (qw * qx + qy * qz);
        g.z() = qw * qw - qx * qx - qy * qy + qz * qz;

        return NormalizeVector(g);
    }

    static void QuaternionToEuler(const Eigen::Quaternionf& q,
                                  float& roll_rad,
                                  float& pitch_rad,
                                  float& yaw_rad)
    {
        const float qw = q.w();
        const float qx = q.x();
        const float qy = q.y();
        const float qz = q.z();

        const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        roll_rad = std::atan2(sinr_cosp, cosr_cosp);

        const float sinp = 2.0f * (qw * qy - qz * qx);
        if (std::fabs(sinp) >= 1.0f)
        {
            pitch_rad = std::copysign(3.14159265358979323846f / 2.0f, sinp);
        }
        else
        {
            pitch_rad = std::asin(sinp);
        }

        const float siny_cosp = 2.0f * (qw * qz + qx * qy);
        const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    }

private:
    Eigen::Quaternionf q_ = Eigen::Quaternionf::Identity();
    Vec3 gyro_bias_ = Vec3::Zero();

    Mat7 P_ = Mat7::Zero();

    float q_attitude_ = 1e-3f;
    float q_bias_ = 1e-5f;
    float r_accel_ = 2e-2f;

    float bias_gain_ = 1e-4f;
};


#else
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Quaternion
    {
        float w = 1.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct EKFAttitudeInput
    {
        // Accelerometer in g
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 1.0f;

        // Gyroscope in rad/s
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;

        // delta time in seconds
        float dt = 0.001f;
    };

    struct EKFAttitudeOutput
    {
        Quaternion q{};

        float roll_rad = 0.0f;
        float pitch_rad = 0.0f;
        float yaw_rad = 0.0f;

        float bgx = 0.0f;
        float bgy = 0.0f;
        float bgz = 0.0f;
    };

    class QuaternionEKF
    {
        public:
            QuaternionEKF()
            {
                Reset();
            }

            inline void Reset()
            {
                q_ = {};
                gyro_bias_ = {};

                ZeroMatrix(P_);
                for (int i = 0; i < 7; ++i)
                {
                    P_[i][i] = 1e-3f;
                }
            }

            inline void SetProcessNoise(float q_attitude, float q_bias)
            {
                q_attitude_ = q_attitude;
                q_bias_ = q_bias;
            }

            inline void SetMeasurementNoise(float r_accel)
            {
                r_accel_ = r_accel;
            }

            inline void Predict(const EKFAttitudeInput& in)
            {
                const float dt = in.dt;
                if (dt <= 0.0f)
                {
                    return;
                }

                // gyro measurement - estimated bias
                const float wx = in.gx - gyro_bias_.x;
                const float wy = in.gy - gyro_bias_.y;
                const float wz = in.gz - gyro_bias_.z;

                // Quaternion derivative
                // q_dot = 0.5 * Omega(w) * q
                Quaternion q_dot;
                q_dot.w = -0.5f * ( wx * this->q_.x + wy * this->q_.y + wz * this->q_.z );
                q_dot.x =  0.5f * ( wx * this->q_.w + wz * this->q_.y - wy * this->q_.z );
                q_dot.y =  0.5f * ( wy * this->q_.w - wz * this->q_.x + wx * this->q_.z );
                q_dot.z =  0.5f * ( wz * this->q_.w + wy * this->q_.x - wx * this->q_.y );

                this->q_.w += q_dot.w * dt;
                this->q_.x += q_dot.x * dt;
                this->q_.y += q_dot.y * dt;
                this->q_.z += q_dot.z * dt;

                NormalizeQuaternion();

                // Very simplified covariance propagation
                // State = [q0 q1 q2 q3 bgx bgy bgz]
                for (int i = 0; i < 4; ++i)
                {
                    P_[i][i] += this->q_attitude_ * dt;
                }

                for (int i = 4; i < 7; ++i)
                {
                    P_[i][i] += this->q_bias_ * dt;
                }
            }

            inline void Update(const EKFAttitudeInput& in)
            {
                // Normalize accel measurement
                Vec3 a_meas{in.ax, in.ay, in.az};
                a_meas = NormalizeVector(a_meas);

                // Predicted gravity direction in body frame
                Vec3 g_pred = PredictGravityBody();

                // Residual: measured gravity - predicted gravity
                Vec3 r;
                r.x = a_meas.x - g_pred.x;
                r.y = a_meas.y - g_pred.y;
                r.z = a_meas.z - g_pred.z;

                // Approximate measurement correction:
                // use cross product between predicted and measured gravity
                // as a small-angle correction
                // Vec3 corr = Cross(g_pred, a_meas);
                Vec3 corr = Cross(a_meas, g_pred);

                // gain from measurement noise
                const float k = 1.0f / (1.0f + this->r_accel_);

                // Small-angle correction to quaternion
                Quaternion dq;
                dq.w = 1.0f;
                dq.x = 0.5f * corr.x * k;
                dq.y = 0.5f * corr.y * k;
                dq.z = 0.5f * corr.z * k;

                this->q_ = QuaternionMultiply(this->q_, dq);
                NormalizeQuaternion();

                // Bias correction (simple form)
                this->gyro_bias_.x += corr.x * this->bias_gain_;
                this->gyro_bias_.y += corr.y * this->bias_gain_;
                // this->gyro_bias_.z += corr.z * this->bias_gain_;

                // Simplified covariance shrink after update
                for (int i = 0; i < 7; ++i)
                {
                    P_[i][i] *= 0.999f;
                }
            }

            inline EKFAttitudeOutput GetOutput() const
            {
                EKFAttitudeOutput out{};
                out.q = q_;
                out.bgx = gyro_bias_.x;
                out.bgy = gyro_bias_.y;
                out.bgz = gyro_bias_.z;

                QuaternionToEuler(q_, out.roll_rad, out.pitch_rad, out.yaw_rad);
                return out;
            }

            inline Quaternion GetQuaternion() const
            {
                return q_;
            }

            inline Vec3 GetGyroBias() const
            {
                return gyro_bias_;
            }

        private:
            static inline void ZeroMatrix(float m[7][7])
            {
                for (int r = 0; r < 7; ++r)
                {
                    for (int c = 0; c < 7; ++c)
                    {
                        m[r][c] = 0.0f;
                    }
                }
            }

            static inline float Norm(const Vec3& v)
            {
                return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            }

            static inline Vec3 NormalizeVector(const Vec3& v)
            {
                const float n = Norm(v);
                if (n <= 1e-6f)
                {
                    return {0.0f, 0.0f, 1.0f};
                }

                return {v.x / n, v.y / n, v.z / n};
            }

            static inline Vec3 Cross(const Vec3& a, const Vec3& b)
            {
                return {
                    a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x
                };
            }

            static inline Quaternion QuaternionMultiply(const Quaternion& a, const Quaternion& b)
            {
                Quaternion out{};
                out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
                out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
                out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
                out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
                return out;
            }

            inline void NormalizeQuaternion()
            {
                const float n = std::sqrt(q_.w * q_.w + q_.x * q_.x + q_.y * q_.y + q_.z * q_.z);
                if (n <= 1e-6f)
                {
                    q_ = {};
                    return;
                }

                q_.w /= n;
                q_.x /= n;
                q_.y /= n;
                q_.z /= n;
            }

            inline Vec3 PredictGravityBody() const
            {
                // World gravity = [0, 0, 1]
                // Rotate into body frame using quaternion
                // Common body-frame predicted gravity formula
                Vec3 g{};
                g.x = 2.0f * (q_.x * q_.z - q_.w * q_.y);
                g.y = 2.0f * (q_.w * q_.x + q_.y * q_.z);
                g.z = q_.w * q_.w - q_.x * q_.x - q_.y * q_.y + q_.z * q_.z;
                return NormalizeVector(g);
            }

            static inline void QuaternionToEuler(const Quaternion& q,
                                                float& roll_rad,
                                                float& pitch_rad,
                                                float& yaw_rad)
            {
                // roll (x-axis rotation)
                const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
                const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
                roll_rad = std::atan2(sinr_cosp, cosr_cosp);

                // pitch (y-axis rotation)
                const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
                if (std::fabs(sinp) >= 1.0f)
                {
                    pitch_rad = std::copysign(3.14159265358979323846f / 2.0f, sinp);
                }
                else
                {
                    pitch_rad = std::asin(sinp);
                }

                // yaw (z-axis rotation)
                const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
                const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
                yaw_rad = std::atan2(siny_cosp, cosy_cosp);
            }

        private:
            // state
            Quaternion q_{};
            Vec3 gyro_bias_{};

            // covariance (kept for EKF-compatible structure)
            float P_[7][7]{};

            // tuning parameters
            float q_attitude_ = 1e-3f;
            float q_bias_ = 1e-5f;
            float r_accel_ = 2e-2f;

            // simple bias adaptation gain
            float bias_gain_ = 1e-4f;
    };
#endif
