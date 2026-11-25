#pragma once
#include <cmath>
#include <cstdlib>
#include <iostream>
inline void require(bool ok, const char* message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}
inline void near(float actual, float expected, float tolerance, const char* message) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        std::cerr << "actual=" << actual << " expected=" << expected << '\n';
        require(false, message);
    }
}
