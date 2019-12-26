#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG 1

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 15 //hz
#define DEBUG_RATE 1

#define K_P 0.16// P constant
#define K_I 0.05 // I constant
#define K_D 0.001 // D constant

// define your robot' specs here
#define MAX_RPM 300 // motor's maximum RPM
#define COUNTS_PER_REV 1650 // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.125// wheel's diameter in meters
#define PWM_BITS 8 // PWM Resolution of the microcontroller
#define BASE_WIDTH 0.6 // width of the plate you are using

// ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 21 // front_A
#define MOTOR1_ENCODER_B 42 // front_B

#define MOTOR3_ENCODER_A 20 // rear_A
#define MOTOR3_ENCODER_B 44 // rear_B

// right side encoders pins
#define MOTOR2_ENCODER_A 19 // front_A
#define MOTOR2_ENCODER_B 46 // front_B

#define MOTOR4_ENCODER_A 18 // rear_A
#define MOTOR4_ENCODER_B 48 // rear_B

//left side motor pins
#define MOTOR1_PWM 2
#define MOTOR1_IN_A 28
#define MOTOR1_IN_B 29

#define MOTOR3_PWM  3
#define MOTOR3_IN_A 30
#define MOTOR3_IN_B 31

//right side motor pins
#define MOTOR2_PWM 4
#define MOTOR2_IN_A 40
#define MOTOR2_IN_B 33

#define MOTOR4_PWM 5
#define MOTOR4_IN_A 34
#define MOTOR4_IN_B 35
#endif
