#pragma once

#include "pid.h"

#include <algorithm>
#include <cmath>
#include <packets.h>

#include  "flight_profile.h"

#if FLIGHT_PROFILE == FLIGHT_PROFILE_TUNE

    // Rate PID
    #define ROLL_RATE_P 0.0360f    //0.08~0.12   //0.0450f
    #define ROLL_RATE_I 0.0030f    //0.04~0.88   //0.0030f
    #define ROLL_RATE_D 0.0002f   //0.001 ~ 0.003    //0.0007f

    #define PITCH_RATE_P 0.0360f   //0.08~0.12  //0.0450f
    #define PITCH_RATE_I 0.0030f   //0.04~0.88  //0.0030f
    #define PITCH_RATE_D 0.0002f  //0.001 ~ 0.003   0.0007f

    #define YAW_RATE_P 0.0600f    //0.05~0.1   //0.130
    #define YAW_RATE_I 0.0013f    //0.01~0.05  //0.020
    #define YAW_RATE_D 0.0000f    //0.000~0.002  //0.000

    // Angle PID
    #define ROLL_ANGLE_P 3.7f     //4.0~6.0
    #define ROLL_ANGLE_I 0.0f     //0.0~0.1
    #define ROLL_ANGLE_D 0.0f     //0.0~0.

    #define PITCH_ANGLE_P 3.7f    //4.0~6.0
    #define PITCH_ANGLE_I 0.0f    //0.0~0.1
    #define PITCH_ANGLE_D 0.0f    //0.0~0

    #define YAW_ANGLE_P 0.120f      //0.0~0.1   //0.80
    #define YAW_ANGLE_I 0.000f      //0.0~0.05
    #define YAW_ANGLE_D 0.000f      //0.0~0.

    // PID limits
    #define ANGLE_PID_MAX_RATE_RAD_S    2.5f //1.5
    #define RATE_PID_MAX_RATE_RAD       1.1f
    #define YAW_RATE_PID_MAX_RATE_RAD   0.35f  //0.25f


    #define RP_ANGLE_INTEGRAL           5.0f
    #define YAW_ANGLE_INTEGRAL          0.2f
    #define RATE_INTEGRAL               2.0f
    #define YAW_RATE_RATE_INTEGRAL      1.0f

    // yaw hold
    #define MAX_YAW_HOLD_RATE_RAD_S    DEG2RAD(75.0f)
    #define YAW_STICK_DEADBAND_RAD_S  DEG2RAD(5.0f)


#elif FLIGHT_PROFILE == FLIGHT_PROFILE_NORMAL


    #define ROLL_RATE_P  0.040f
    #define ROLL_RATE_I  0.003f
    #define ROLL_RATE_D  0.000f

    #define PITCH_RATE_P 0.050f
    #define PITCH_RATE_I 0.003f
    #define PITCH_RATE_D 0.000f

    #define YAW_RATE_P   0.050f
    #define YAW_RATE_I   0.010f
    #define YAW_RATE_D   0.000f

    // Angle PID
    #define ROLL_ANGLE_P 3.5f     //4.0~6.0
    #define ROLL_ANGLE_I 0.0f     //0.0~0.1
    #define ROLL_ANGLE_D 0.0f     //0.0~0.

    #define PITCH_ANGLE_P 3.5f    //4.0~6.0
    #define PITCH_ANGLE_I 0.0f    //0.0~0.1
    #define PITCH_ANGLE_D 0.0f    //0.0~0

    #define YAW_ANGLE_P 0.0f      //0.0~0.1
    #define YAW_ANGLE_I 0.0f      //0.0~0.05
    #define YAW_ANGLE_D 0.0f      //0.0~0.

    // PID limits
    #define ANGLE_PID_MAX_RATE_RAD_S    2.5f
    #define RATE_PID_MAX_RATE_RAD       0.5f
    #define YAW_RATE_PID_MAX_RATE_RAD   0.30f


    #define ANGLE_INTEGRAL              5.0f
    #define RATE_INTEGRAL               2.0f
    #define YAW_RATE_RATE_INTEGRAL      1.0f

    // yaw hold
    #define MAX_YAW_HOLD_RATE_RAD_S DEG2RAD(20.0f)
    #define YAW_STICK_DEADBAND DEG2RAD(3.0f)



#else
    #error "Unknown FLIGHT_PROFILE"
#endif