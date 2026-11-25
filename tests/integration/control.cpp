#include "check.h"
#include "flight_host.h"
#include "ekf.h"
#include "motor_mixer.h"
int main() {
    QuaternionEKF estimator;
    EKFAttitudeInput imu{};
    estimator.Predict(imu);
    estimator.Update(imu);
    auto q = estimator.GetOutput();
    AttitudeState state{};
    state.roll_rad=q.roll_rad;
    state.pitch_rad=q.pitch_rad;
    state.yaw_rad=q.yaw_rad;
    FlightPIDController pid;
    require(pid.Initialize()==ESP_OK, "PID initialization");
    AttitudeTarget target{};
    target.throttle=0.5f;
    target.roll_rate_rad_s=0.2f;
    auto u=pid.Update(target,state,0.001f);
    require(u.roll>0, "positive roll response");
    auto m=MixMotorsWithMultiplier({u.throttle,u.roll,u.pitch,u.yaw},1,1,1,1);
    MixNormalize(m);
    require(m.m1>m.m2 && m.m4>m.m3, "roll motor differential");
    near((m.m1+m.m2+m.m3+m.m4)/4, 0.5f, 1e-5f, "mean thrust");
}
