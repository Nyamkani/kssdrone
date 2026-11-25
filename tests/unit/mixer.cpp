#include "check.h"
#include "motor_mixer.h"
int main() {
    auto o = MixMotorsWithMultiplier({0.5f, 0.1f, 0, 0}, 1, 1, 1, 1);
    near(o.m1, 0.6f, 1e-6f, "roll FL");
    near(o.m2, 0.4f, 1e-6f, "roll FR");
    near(o.m3, 0.4f, 1e-6f, "roll RR");
    near(o.m4, 0.6f, 1e-6f, "roll RL");
    o = MixMotorsWithMultiplier({0.5f, 0, 0, 0.1f}, 1, 1, 1, 1);
    near(o.m1, 0.6f, 1e-6f, "active mixer yaw M1");
    near(o.m2, 0.4f, 1e-6f, "active mixer yaw M2");
    o = {1.4f, -0.3f, 0.5f, 0.9f};
    MixNormalize(o);
    for (float x : {o.m1, o.m2, o.m3, o.m4})
        require(x >= 0 && x <= 1, "normalized output bounds");
}
