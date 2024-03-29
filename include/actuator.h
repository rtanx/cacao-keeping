#ifndef ACTUATOR_H
#define ACTUATOR_H
#include <Arduino.h>

#define MOTOR_LPWM_PIN GPIO_NUM_32
#define MOTOR_RPWM_PIN GPIO_NUM_33
#define MOTOR_LEN_PIN GPIO_NUM_25
#define MOTOR_REN_PIN GPIO_NUM_26

#define MOTOR_L_CHANNEL 15
#define MOTOR_R_CHANNEL 14

#define MOTOR_FREQUENCY 1000
#define MOTOR_RESOLUTION 8

enum MotorDirection {
    CW,
    CCW
};

void setupMotorDriver();
void runMotor(enum MotorDirection dir, uint8_t speed);
void enableMotor(bool cw, bool ccw);
#endif