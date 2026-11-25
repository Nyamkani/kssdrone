#include "check.h"
#include "pid.h"
#include "motor_mixer.h"
int main() {
    // Explicit baseline for active mixer sign convention, not legacy MixMotors.
    auto m=MixMotorsWithMultiplier({0.5f,0.1f,0.05f,0.025f},1,1,1,1);
    near(m.m1,0.575f,1e-6f,"baseline M1");
    near(m.m2,0.325f,1e-6f,"baseline M2");
    near(m.m3,0.475f,1e-6f,"baseline M3");
    near(m.m4,0.625f,1e-6f,"baseline M4");
    PIDController2 pid({0.2f,0,0},{-1,1,-1,1});
    near(pid.Update(0.5f,0.1f,0.002f,false,0.5f,1),0.04f,1e-6f,"P scaling baseline");
    pid.Reset();
    near(pid.Update(0,0,0.002f),0,1e-6f,"reset baseline");
}
