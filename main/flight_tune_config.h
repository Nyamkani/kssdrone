#pragma once
#include  "flight_profile.h"


#if FLIGHT_PROFILE == FLIGHT_PROFILE_TUNE


    #define MAX_ROLL_RATE_RAD_S   DEG2RAD(500.0f)
    #define MAX_PITCH_RATE_RAD_S  DEG2RAD(500.0f)
    #define MAX_YAW_RATE_RAD_S    DEG2RAD(300.0f)

    #define MAX_ROLL_ANGLE_RAD    DEG2RAD(40.0f)
    #define MAX_PITCH_ANGLE_RAD   DEG2RAD(40.0f)

    // thrust
    #define THRUST_EXPO                0.5f

    // PID authority
    #define PID_MIN_AUTHORITY_RP       0.5f
    #define PID_MIN_AUTHORITY_YAW      1.0f

    // TPA
    #define TPA_BREAKPOINT             0.55f
    #define TPA_RATE_RP                0.25f
    #define TPA_RATE_YAW               0.05f

    // yaw priority / roll-pitch protection
    #define YAW_RP_PROTECT_START     0.08f
    #define YAW_RP_PROTECT_FULL      0.25f
    #define YAW_RP_PROTECT_MAX_CUT   0.35f
    #define YAW_RP_PROTECT_MIN_SCALE 0.65f
    
    // hover yaw hold boost
    #define YAW_HOVER_BOOST_MAX       1.25f
    #define YAW_HOVER_THR_RATE_REF    0.20f
    #define YAW_HOVER_RP_ACTIVITY_REF 0.08f

    // yaw bias
    // #define YAW_OUTPUT_BIAS -0.022f  // negative = CCW 억제 / CW 보정
    #define YAW_OUTPUT_BIAS_COEF  0.0f
    #define YAW_MIX_BIAS 0.0000f   // - = CCW 보정, + = CW 보정
    #define YAW_OUTPUT_BIAS_AIRBORNE  0.0f // negative = CCW 억제, positive = CCW 유도

    // yaw throttle down FF
    #define YAW_THR_DOWN_FF_GAIN  0.0000f //0.0010f
    #define YAW_THR_DOWN_FF_MAX   0.0000f //0.0150f

    // climb dynamic attitude trim
    #define CLIMB_ROLL_TRIM_MAX        DEG2RAD(0.00f)
    #define CLIMB_PITCH_TRIM_MAX       DEG2RAD(0.00f)   

    #define CLIMB_TRIM_THR_RATE_START  0.05f
    #define CLIMB_TRIM_THR_RATE_FULL   0.35f
    #define CLIMB_TRIM_MIN_THROTTLE    0.35f

    #define CLIMB_TRIM_LPF_ALPHA       0.05f

    // takeoff dynamic attitude trim
    #define TAKEOFF_ROLL_TRIM_MAX        DEG2RAD(0.00f)
    #define TAKEOFF_PITCH_TRIM_MAX       DEG2RAD(0.00f)

    #define TAKEOFF_TRIM_THR_MIN         0.10f

    #define TAKEOFF_TRIM_THR_RATE_START  0.05f
    #define TAKEOFF_TRIM_THR_RATE_FULL   0.35f

    #define TAKEOFF_TRIM_ATTACK_ALPHA    0.15f
    #define TAKEOFF_TRIM_RELEASE_ALPHA   0.08f


    // throttle ramp
    #define THROTTLE_RAMP_UP_RATE           1.2f
    #define TAKEOFF_THROTTLE_RAMP_UP_RATE   0.8f
    #define THROTTLE_RAMP_DOWN_RATE         1.0f

    // airborne detect
    #define AIRBORNE_THROTTLE_ENTER     0.35f
    #define AIRBORNE_THROTTLE_EXIT      0.25f
    #define AIRBORNE_EXIT_CONFIRM_TIME  2.50f
    #define AIRBORNE_CONFIRM_TIME       0.15f


    // auto trim
    #define AUTO_TRIM_TIME             3.0f
    #define AUTO_TRIM_ROLL_GAIN        0.03f
    #define AUTO_TRIM_PITCH_GAIN       0.02f

    // IMU LPF
    #define LPF_GYRO_ALPHA             0.35f
    #define LPF_ACC_ALPHA              0.15f

    // vertical estimator / d amping
    #define AZ_LPF_ALPHA               0.08f
    #define VZ_LEAK_PER_SEC            1.5f   //1.5f
    #define K_VZ                       0.020f  //0.005f

    // VZ based attitude compensations
    #define K_ROLL_VZ_COMP   (0.000f)   //roll - left, + right
    #define K_PITCH_VZ_COMP  (0.000f)   //pitch - backward, + forward
    #define K_YAW_VZ_COMP    (0.000f)   //cw -, ccw +

    // voltage compensation
    #define BATTERY_VOLTAGE_REF        8.4f
    #define BATTERY_VOLTAGE_MIN_VALID  6.0f

    #define THRUST_VOLT_COMP_GAIN      0.30f
    #define V_SCALE_MAX                1.15f
    #define VOLT_COMP_BLEND_RANGE      0.40f

    #define AXIS_VOLT_COMP_GAIN_RP   0.20f
    #define AXIS_VOLT_COMP_GAIN_YAW  0.10f

    #define VOLT_COMP_START_THROTTLE  AIRBORNE_THROTTLE_ENTER 
    #define VOLT_COMP_FULL_THROTTLE   0.55f

    // motor output limit
    #define HIGH_THR_LIMIT_START     0.80f
    #define HIGH_THR_LIMIT_FULL      1.00f
    #define HIGH_THR_MAX_CUT         0.06f   // 최대 6% 감소

    // mixer bias
    #define FRONT_THRUST_BIAS          1.000f//0.985f
    #define REAR_THRUST_BIAS           1.000f//1.015f
    #define LEFT_THRUST_BIAS           1.000f//0.985f 1.007
    #define RIGHT_THRUST_BIAS          1.000f//1.015f 0.993

    //motor bias
    #define M1_FL_BIAS  1.000f
    #define M2_FR_BIAS  1.000f  //0.990
    #define M3_RR_BIAS  1.000f
    #define M4_RL_BIAS  1.000f  //1.010

    //slew limit
    // #define ROLL_SLEW_RATE_UP   100.0f          // rad/s
    // #define ROLL_SLEW_RATE_DOWN         100.0f   // rad/s
    #define TARGET_ANGLE_SLEW_RATE      DEG2RAD(1500.0f)  //500
    #define TARGET_YAW_SLEW_RATE        DEG2RAD(2000.0f)  //1000 
    #define YAW_STICK_DEADBAND          DEG2RAD(3.0f)
    
    // ground blend
    #define GROUND_BLEND_RANGE         0.35f

    // horizontal pseudo velocity estimator
    #define AXY_LPF_ALPHA       0.05f
    #define VXY_LEAK_PER_SEC    2.0f

    // velocity based attitude compensation
    #define K_VX_TO_PITCH       DEG2RAD(0.0f)
    #define K_VY_TO_ROLL        DEG2RAD(0.0f)   
    #define VXY_COMP_MAX        DEG2RAD(2.0f)

#elif FLIGHT_PROFILE == FLIGHT_PROFILE_INDOOR_STABLE

    #define MAX_ROLL_RATE_RAD_S   DEG2RAD(50.0f)
    #define MAX_PITCH_RATE_RAD_S  DEG2RAD(50.0f)
    #define MAX_YAW_RATE_RAD_S    DEG2RAD(60.0f)

    #define MAX_ROLL_ANGLE_RAD    DEG2RAD(20.0f)
    #define MAX_PITCH_ANGLE_RAD   DEG2RAD(20.0f)

    // thrust
    #define THRUST_EXPO                0.5f

    // PID authority
    #define PID_MIN_AUTHORITY_RP       0.5f
    #define PID_MIN_AUTHORITY_YAW      1.0f

    // TPA
    #define TPA_BREAKPOINT             0.55f
    #define TPA_RATE_RP                0.25f
    #define TPA_RATE_YAW               0.05f

    // yaw priority / roll-pitch protection
    #define YAW_RP_PROTECT_START     0.08f
    #define YAW_RP_PROTECT_FULL      0.25f
    #define YAW_RP_PROTECT_MAX_CUT   0.35f
    #define YAW_RP_PROTECT_MIN_SCALE 0.65f
    
    // hover yaw hold boost
    #define YAW_HOVER_BOOST_MAX       1.25f
    #define YAW_HOVER_THR_RATE_REF    0.20f
    #define YAW_HOVER_RP_ACTIVITY_REF 0.08f

    // yaw bias
    // #define YAW_OUTPUT_BIAS -0.022f  // negative = CCW 억제 / CW 보정
    #define YAW_OUTPUT_BIAS_COEF  0.0f
    #define YAW_MIX_BIAS 0.0000f   // - = CCW 보정, + = CW 보정
    #define YAW_OUTPUT_BIAS_AIRBORNE  0.0f // negative = CCW 억제, positive = CCW 유도

    // yaw throttle down FF
    #define YAW_THR_DOWN_FF_GAIN  0.0000f //0.0010f
    #define YAW_THR_DOWN_FF_MAX   0.0000f //0.0150f

    // climb dynamic attitude trim
    #define CLIMB_ROLL_TRIM_MAX        DEG2RAD(0.00f)
    #define CLIMB_PITCH_TRIM_MAX       DEG2RAD(0.00f)   

    #define CLIMB_TRIM_THR_RATE_START  0.05f
    #define CLIMB_TRIM_THR_RATE_FULL   0.35f
    #define CLIMB_TRIM_MIN_THROTTLE    0.35f

    #define CLIMB_TRIM_LPF_ALPHA       0.05f

    // takeoff dynamic attitude trim
    #define TAKEOFF_ROLL_TRIM_MAX        DEG2RAD(0.00f)
    #define TAKEOFF_PITCH_TRIM_MAX       DEG2RAD(0.00f)

    #define TAKEOFF_TRIM_THR_MIN         0.10f

    #define TAKEOFF_TRIM_THR_RATE_START  0.05f
    #define TAKEOFF_TRIM_THR_RATE_FULL   0.35f

    #define TAKEOFF_TRIM_ATTACK_ALPHA    0.15f
    #define TAKEOFF_TRIM_RELEASE_ALPHA   0.08f


    // throttle ramp
    #define THROTTLE_RAMP_UP_RATE           1.2f
    #define TAKEOFF_THROTTLE_RAMP_UP_RATE   0.8f
    #define THROTTLE_RAMP_DOWN_RATE         1.0f

    // airborne detect
    #define AIRBORNE_THROTTLE_ENTER     0.35f
    #define AIRBORNE_THROTTLE_EXIT      0.25f
    #define AIRBORNE_EXIT_CONFIRM_TIME  2.50f
    #define AIRBORNE_CONFIRM_TIME       0.15f


    // auto trim
    #define AUTO_TRIM_TIME             3.0f
    #define AUTO_TRIM_ROLL_GAIN        0.03f
    #define AUTO_TRIM_PITCH_GAIN       0.02f

    // IMU LPF
    #define LPF_GYRO_ALPHA             0.35f
    #define LPF_ACC_ALPHA              0.15f

    // vertical estimator / d amping
    #define AZ_LPF_ALPHA               0.08f
    #define VZ_LEAK_PER_SEC            1.5f   //1.5f
    #define K_VZ                       0.020f  //0.005f

    // VZ based attitude compensations
    #define K_ROLL_VZ_COMP   (0.000f)   //roll - left, + right
    #define K_PITCH_VZ_COMP  (0.000f)   //pitch - backward, + forward
    #define K_YAW_VZ_COMP    (0.000f)   //cw -, ccw +

    // voltage compensation
    #define BATTERY_VOLTAGE_REF        8.4f
    #define BATTERY_VOLTAGE_MIN_VALID  6.0f

    #define THRUST_VOLT_COMP_GAIN      0.30f
    #define V_SCALE_MAX                1.15f
    #define VOLT_COMP_BLEND_RANGE      0.40f

    #define AXIS_VOLT_COMP_GAIN_RP   0.20f
    #define AXIS_VOLT_COMP_GAIN_YAW  0.10f

    #define VOLT_COMP_START_THROTTLE  AIRBORNE_THROTTLE_ENTER 
    #define VOLT_COMP_FULL_THROTTLE   0.55f

    // motor output limit
    #define HIGH_THR_LIMIT_START     0.80f
    #define HIGH_THR_LIMIT_FULL      1.00f
    #define HIGH_THR_MAX_CUT         0.06f   // 최대 6% 감소

    // mixer bias
    #define FRONT_THRUST_BIAS          1.000f//0.985f
    #define REAR_THRUST_BIAS           1.000f//1.015f
    #define LEFT_THRUST_BIAS           1.000f//0.985f 1.007
    #define RIGHT_THRUST_BIAS          1.000f//1.015f 0.993

    //motor bias
    #define M1_FL_BIAS  1.000f
    #define M2_FR_BIAS  1.000f  //0.990
    #define M3_RR_BIAS  1.000f
    #define M4_RL_BIAS  1.000f  //1.010

    //slew limit
    // #define ROLL_SLEW_RATE_UP   100.0f          // rad/s
    // #define ROLL_SLEW_RATE_DOWN         100.0f   // rad/s
    #define TARGET_ANGLE_SLEW_RATE      DEG2RAD(1500.0f)  //500
    #define TARGET_YAW_SLEW_RATE        DEG2RAD(2000.0f)  //1000 
    #define YAW_STICK_DEADBAND          DEG2RAD(3.0f)
    
    // ground blend
    #define GROUND_BLEND_RANGE         0.35f

    // horizontal pseudo velocity estimator
    #define AXY_LPF_ALPHA       0.05f
    #define VXY_LEAK_PER_SEC    2.0f

    // velocity based attitude compensation
    #define K_VX_TO_PITCH       DEG2RAD(0.0f)
    #define K_VY_TO_ROLL        DEG2RAD(0.0f)   
    #define VXY_COMP_MAX        DEG2RAD(2.0f)



#elif FLIGHT_PROFILE == FLIGHT_PROFILE_NORMAL

       // thrust
    #define THRUST_EXPO                0.5f

    // PID authority
    #define PID_MIN_AUTHORITY_RP       0.5f
    #define PID_MIN_AUTHORITY_YAW      1.0f

    // TPA
    #define TPA_BREAKPOINT             0.55f
    #define TPA_RATE_RP                0.25f
    #define TPA_RATE_YAW               0.05f

    // yaw priority / roll-pitch protection
    #define YAW_RP_PROTECT_START     0.08f
    #define YAW_RP_PROTECT_FULL      0.25f
    #define YAW_RP_PROTECT_MAX_CUT   0.45f
    #define YAW_RP_PROTECT_MIN_SCALE 0.55f
    
    // hover yaw hold boost
    #define YAW_HOVER_BOOST_MAX       1.25f
    #define YAW_HOVER_THR_RATE_REF    0.20f
    #define YAW_HOVER_RP_ACTIVITY_REF 0.08f

    // yaw bias
    // #define YAW_OUTPUT_BIAS -0.022f  // negative = CCW 억제 / CW 보정
    #define YAW_OUTPUT_BIAS_COEF  0.0f
    #define YAW_MIX_BIAS 0.0012f   // - = CCW 보정, + = CW 보정
    #define YAW_OUTPUT_BIAS_AIRBORNE  0.0f // negative = CCW 억제, positive = CCW 유도

    // yaw throttle down FF
    #define YAW_THR_DOWN_FF_GAIN  0.0010f //0.0008f
    #define YAW_THR_DOWN_FF_MAX   0.0150f //0.0100f

    // throttle ramp
    #define THROTTLE_RAMP_UP_RATE           1.2f
    #define TAKEOFF_THROTTLE_RAMP_UP_RATE   0.8f
    #define THROTTLE_RAMP_DOWN_RATE         1.0f

    // airborne detect
    #define AIRBORNE_THROTTLE_ENTER     0.35f
    #define AIRBORNE_THROTTLE_EXIT      0.25f
    #define AIRBORNE_EXIT_CONFIRM_TIME  0.35f
    #define AIRBORNE_CONFIRM_TIME       0.12f


    // auto trim
    #define AUTO_TRIM_TIME             3.0f
    #define AUTO_TRIM_ROLL_GAIN        0.03f
    #define AUTO_TRIM_PITCH_GAIN       0.02f

    // IMU LPF
    #define LPF_GYRO_ALPHA             0.70f
    #define LPF_ACC_ALPHA              0.35f

    // vertical estimator / damping
    #define AZ_LPF_ALPHA               0.08f
    #define VZ_LEAK_PER_SEC            1.5f   //1.5f
    #define K_VZ                       0.020f  //0.005f

    // VZ based attitude compensations
    #define K_ROLL_VZ_COMP   (0.000f)   //roll - left, + right
    #define K_PITCH_VZ_COMP  (0.000f)   //pitch - backward, + forward
    #define K_YAW_VZ_COMP    (0.000f)   //cw -, ccw +

    // voltage compensation
    #define BATTERY_VOLTAGE_REF        8.4f
    #define BATTERY_VOLTAGE_MIN_VALID  6.0f

    #define THRUST_VOLT_COMP_GAIN      0.30f
    #define V_SCALE_MAX                1.15f
    #define VOLT_COMP_BLEND_RANGE      0.40f

    #define AXIS_VOLT_COMP_GAIN_RP   0.20f
    #define AXIS_VOLT_COMP_GAIN_YAW  0.10f

    #define VOLT_COMP_START_THROTTLE  AIRBORNE_THROTTLE_ENTER 
    #define VOLT_COMP_FULL_THROTTLE   0.55f

    // motor output limit
    #define HIGH_THR_LIMIT_START     0.80f
    #define HIGH_THR_LIMIT_FULL      1.00f
    #define HIGH_THR_MAX_CUT         0.06f   // 최대 6% 감소

    // mixer bias
    #define FRONT_THRUST_BIAS          1.000f//0.985f
    #define REAR_THRUST_BIAS           1.000f//1.015f
    #define LEFT_THRUST_BIAS           1.000f//0.985f 1.007
    #define RIGHT_THRUST_BIAS          1.000f//1.015f 0.993

    //motor bias
    #define M1_FL_BIAS  1.000f
    #define M2_FR_BIAS  1.000f  //0.990
    #define M3_RR_BIAS  1.000f
    #define M4_RL_BIAS  1.000f  //1.010

    //slew limit
    // #define ROLL_SLEW_RATE_UP   100.0f          // rad/s
    // #define ROLL_SLEW_RATE_DOWN         100.0f   // rad/s
    #define TARGET_ANGLE_SLEW_RATE      DEG2RAD(1500.0f)  //500
    #define TARGET_YAW_SLEW_RATE        DEG2RAD(2000.0f)  //1000 
    #define YAW_STICK_DEADBAND          DEG2RAD(3.0f)
    
    // ground blend
    #define GROUND_BLEND_RANGE         0.35f

    // horizontal pseudo velocity estimator
    #define AXY_LPF_ALPHA       0.05f
    #define VXY_LEAK_PER_SEC    2.0f

    // velocity based attitude compensation
    #define K_VX_TO_PITCH       DEG2RAD(0.0f)
    #define K_VY_TO_ROLL        DEG2RAD(0.0f)   
    #define VXY_COMP_MAX        DEG2RAD(2.0f)


#else
    #error "Unknown FLIGHT_PROFILE"
#endif