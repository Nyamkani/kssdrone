#pragma once

#include <algorithm>

// M1: Front Left
// M2: Front Right
// M3: Rear Right
// M4: Rear Left

//throttle -> Total thrust
//roll -> left/right tilt
//pitch -> forward/backward tilt
//yaw -> rotation around vertical axis

/*
Motor layout (X configuration):

          Front
            ↑
            |

        M1       M2

        M4       M3
*/
/*
M1  CCW  -> real -> m2
M2  CW   -> real -> m4
M3  CCW  -> real -> m1
M4  CW   -> readl -> m3
*/

//ex ) roll -> right tilt -> M1, M4 increase, M2, M3 decrease
//ex ) pitch -> backward tilt -> M1, M2 increase, M3, M4 decrease
//ex ) yaw -> CCW rotation -> M1, M3 increase, M2, M4 decrease

/*
M1 = throttle + roll - pitch + yaw
M2 = throttle - roll - pitch - yaw
M3 = throttle - roll + pitch + yaw
M4 = throttle + roll + pitch - yaw
*/

struct MixerInput 
{
    float throttle; // 0.0 to 1.0
    float roll;     // -1.0 to 1.0
    float pitch;    // -1.0 to 1.0
    float yaw;      // -1.0 to 1.0
};

struct MixerOutput
{
    float m1; // Motor 1 output
    float m2; // Motor 2 output
    float m3; // Motor 3 output
    float m4; // Motor 4 output
};
// static constexpr float ROLL_LEFT_SCALE = 1.00f;
// static constexpr float ROLL_RIGHT_SCALE  = 1.00f;

// static constexpr float PITCH_FRONT_SCALE = 1.00f;
// static constexpr float PITCH_REAR_SCALE  = 1.00f;

// static constexpr float FRONT_THRUST_BIAS = 0.916f; 
// static constexpr float REAR_THRUST_BIAS  = 1.084f;

// static constexpr float LEFT_THRUST_BIAS  = 1.00f;
// static constexpr float RIGHT_THRUST_BIAS = 1.00f;

static constexpr float ROLL_LEFT_SCALE = 1.00f;
static constexpr float ROLL_RIGHT_SCALE  = 1.00f;

static constexpr float PITCH_FRONT_SCALE = 1.00f;
static constexpr float PITCH_REAR_SCALE  = 1.00f;

inline MixerOutput MixMotors(const MixerInput& input)
{
    MixerOutput output;

    // Basic mixing algorithm for an X configuration quadcopter
    output.m1 = (input.throttle
                - input.pitch * PITCH_FRONT_SCALE
                + input.roll  * ROLL_LEFT_SCALE
                - input.yaw); // FL (CCW)

    output.m2 = (input.throttle
                - input.pitch * PITCH_FRONT_SCALE
                - input.roll  * ROLL_RIGHT_SCALE
                + input.yaw); // FR (CW)

    output.m3 = (input.throttle
                + input.pitch * PITCH_REAR_SCALE
                - input.roll  * ROLL_RIGHT_SCALE
                - input.yaw); // RR (CCW)

    output.m4 = (input.throttle
                + input.pitch * PITCH_REAR_SCALE
                + input.roll  * ROLL_LEFT_SCALE
                + input.yaw); // RL (CW)

    ////alternative mixing with front/rear bias
    // output.m1 = (input.throttle * m1_multiplier
    //             - input.pitch * PITCH_FRONT_SCALE
    //             + input.roll  * ROLL_LEFT_SCALE
    //             - input.yaw); // FL (CCW)

    // output.m2 = (input.throttle * m2_multiplier
    //             - input.pitch * PITCH_FRONT_SCALE
    //             - input.roll  * ROLL_RIGHT_SCALE
    //             + input.yaw); // FR (CW)

    // output.m3 = (input.throttle * m3_multiplier
    //             + input.pitch * PITCH_REAR_SCALE
    //             - input.roll  * ROLL_RIGHT_SCALE
    //             - input.yaw); // RR (CCW)

    // output.m4 = (input.throttle * m4_multiplier
    //             + input.pitch * PITCH_REAR_SCALE
    //             + input.roll  * ROLL_LEFT_SCALE
    //             + input.yaw); // RL (CW)

    return output;
}

inline MixerOutput MixMotorsWithMultiplier(const MixerInput& input,
                                        const float& m1_multiplier,
                                        const float& m2_multiplier,
                                        const float& m3_multiplier,
                                        const float& m4_multiplier)
{
    MixerOutput output;

    // Basic mixing algorithm for an X configuration quadcopter
    output.m1 = (input.throttle
                - input.pitch * PITCH_FRONT_SCALE
                + input.roll  * ROLL_LEFT_SCALE
                + input.yaw)* m1_multiplier; // FL (CW)

    output.m2 = (input.throttle
                - input.pitch * PITCH_FRONT_SCALE
                - input.roll  * ROLL_RIGHT_SCALE
                - input.yaw)* m2_multiplier; // FR (CCW)

    output.m3 = (input.throttle
                + input.pitch * PITCH_REAR_SCALE
                - input.roll  * ROLL_RIGHT_SCALE
                + input.yaw)* m3_multiplier; // RR (CW)

    output.m4 = (input.throttle
                + input.pitch * PITCH_REAR_SCALE
                + input.roll  * ROLL_LEFT_SCALE
                - input.yaw)* m4_multiplier; // RL (CCwW)

    return output;
}




inline float Max4(float a, float b, float c, float d)
{
    return std::max(std::max(a, b), std::max(c, d));
}

inline float Min4(float a, float b, float c, float d)
{
    return std::min(std::min(a, b), std::min(c, d));
}


// inline void MixNormalize(MixerOutput& output, float min_output = 0.0f, float max_output = 1.0f)
// {
//     float max_motor = Max4(output.m1, output.m2, output.m3, output.m4);
//     float min_motor = Min4(output.m1, output.m2, output.m3, output.m4);

//     if (max_motor > max_output)
//     {
//         float scale = max_output / max_motor;
//         output.m1 *= scale;
//         output.m2 *= scale;
//         output.m3 *= scale;
//         output.m4 *= scale;
//     }

//     if (min_motor < min_output)
//     {
//         float offset = min_output - min_motor;
//         output.m1 += offset;
//         output.m2 += offset;
//         output.m3 += offset;
//         output.m4 += offset;
//     }

//     // After normalization, ensure outputs are still within bounds
//     output.m1 = std::clamp(output.m1, min_output, max_output);
//     output.m2 = std::clamp(output.m2, min_output, max_output);
//     output.m3 = std::clamp(output.m3, min_output, max_output);
//     output.m4 = std::clamp(output.m4, min_output, max_output);
    
//     return;
// }

inline void MixNormalize(MixerOutput& output,
                         float min_output = 0.0f,
                         float max_output = 1.0f)
{
    float max_motor = Max4(output.m1, output.m2, output.m3, output.m4);
    float min_motor = Min4(output.m1, output.m2, output.m3, output.m4);

    if (max_motor > max_output)
    {
        const float offset = max_motor - max_output;

        output.m1 -= offset;
        output.m2 -= offset;
        output.m3 -= offset;
        output.m4 -= offset;

        min_motor = Min4(output.m1, output.m2, output.m3, output.m4);
    }

    if (min_motor < min_output)
    {
        const float offset = min_output - min_motor;

        output.m1 += offset;
        output.m2 += offset;
        output.m3 += offset;
        output.m4 += offset;
    }

    output.m1 = std::clamp(output.m1, min_output, max_output);
    output.m2 = std::clamp(output.m2, min_output, max_output);
    output.m3 = std::clamp(output.m3, min_output, max_output);
    output.m4 = std::clamp(output.m4, min_output, max_output);
}