#include "check.h"
#include "pid.h"
#include <iostream>
int main() {
    // Illustrative first-order single-axis plant, not a calibrated drone model.
    constexpr float dt=0.001f, target=0.5f;
    PIDController2 controller({0.8f,0,0},{-1,1,-1,1});
    float rate=0;
    for (int n=0; n<3000; ++n) {
        const float u=controller.Update(target,rate,dt);
        rate += dt*(10*u-rate);
        require(std::isfinite(rate) && std::fabs(rate)<2, "bounded plant");
    }
    std::cout << "final simulated rate=" << rate << '\n';
    near(rate, 4.0f/9.0f, 0.005f, "first-order closed-loop steady state");
}
