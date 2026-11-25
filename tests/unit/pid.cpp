#include "check.h"
#include "pid.h"
int main() {
    PIDController p({0.5f, 0.0f, 0.0f}, {-1, 1, -2, 2});
    near(p.Update(10, 8, 0.01f), 1, 1e-6f, "P output");
    near(p.Update(10, 0, 0.01f), 2, 1e-6f, "positive saturation");
    near(p.Update(-10, 0, 0.01f), -2, 1e-6f, "negative saturation");
    near(p.Update(10, 0, 0), 0, 1e-6f, "invalid dt");
    PIDController i({0, 1, 0}, {-1, 1, -2, 2});
    near(i.Update(1, 0, 0.1f), 0.1f, 1e-6f, "integral");
    near(i.Update(1, 0, 0.1f, true), 0.1f, 1e-6f, "anti windup");
    i.Reset();
    near(i.GetIntegral(), 0, 1e-6f, "reset");
    PIDController2 p2({0.5f, 0, 0}, {-1, 1, -2, 2});
    near(p2.Update(10, 8, 0.01f), 1, 1e-6f, "PID2 P output");
    near(p2.Update(10, 0, 0.01f), 2, 1e-6f, "PID2 saturation");
    near(p2.Update(10, 0, 0), 0, 1e-6f, "PID2 invalid dt");
}
