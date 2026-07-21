#pragma once

//for seeed xiao esp32s3

/*
M1  CCW  -> real -> m2
M2  CW   -> real -> m4
M3  CCW  -> real -> m1
M4  CW   -> real -> m3
*/

// ESC outputs
#define PIN_ESC1        GPIO_NUM_3  //d1   FL  ccw
#define PIN_ESC2        GPIO_NUM_5  //d2   FR  cw
#define PIN_ESC3        GPIO_NUM_4  //d3   RR  ccw
#define PIN_ESC4        GPIO_NUM_2  //d4   RL  cw

// Indicators
#define PIN_BUZZER_LED      GPIO_NUM_6  //d5

// IMU SPI
#define PIN_IMU_CS      GPIO_NUM_7  //d8
#define PIN_IMU_SCLK    GPIO_NUM_8  //d9
#define PIN_IMU_MISO    GPIO_NUM_9  //d10
#define PIN_IMU_MOSI    GPIO_NUM_44 //d7
#define PIN_IMU_INT     GPIO_NUM_43 //d6

// Battery ADC
#define PIN_BAT_ADC     GPIO_NUM_1    //d0




