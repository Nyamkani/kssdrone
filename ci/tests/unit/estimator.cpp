#include "check.h"
#include "ekf.h"
int main() {
    QuaternionEKF estimator;
    EKFAttitudeInput in{};
    for (int n=0; n<100; ++n) {
        estimator.Predict(in);
        estimator.Update(in);
    }
    auto out = estimator.GetOutput();
    near(out.roll_rad, 0, 1e-4f, "stationary roll");
    near(out.pitch_rad, 0, 1e-4f, "stationary pitch");
    near(out.yaw_rad, 0, 1e-4f, "stationary yaw");
    in.gz = 0.5f;
    for (int n=0; n<1000; ++n) estimator.Predict(in);
    out = estimator.GetOutput();
    near(out.yaw_rad, 0.5f, 0.002f, "gyro yaw integration");
    near(out.q.w*out.q.w + out.q.x*out.q.x + out.q.y*out.q.y + out.q.z*out.q.z, 1, 1e-5f, "quaternion norm");
}
