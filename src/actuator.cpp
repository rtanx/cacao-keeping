#include "actuator.h"

bool motorREN = false, motorLEN = false;

void setupMotorDriver() {
    ledcSetup(MOTOR_R_CHANNEL, MOTOR_FREQUENCY, MOTOR_RESOLUTION);
    ledcSetup(MOTOR_L_CHANNEL, MOTOR_FREQUENCY, MOTOR_RESOLUTION);

    ledcAttachPin(MOTOR_RPWM_PIN, MOTOR_R_CHANNEL);
    ledcAttachPin(MOTOR_LPWM_PIN, MOTOR_L_CHANNEL);

    pinMode(MOTOR_LEN_PIN, OUTPUT);
    pinMode(MOTOR_REN_PIN, OUTPUT);
}

void enableMotor(bool cw, bool ccw) {
    digitalWrite(MOTOR_REN_PIN, cw);
    digitalWrite(MOTOR_LEN_PIN, ccw);
    motorREN = cw;
    motorLEN = ccw;
}

void runMotor(enum MotorDirection dir, uint8_t speed) {
    if (motorREN && (dir == CW)) {
        ledcWrite(MOTOR_L_CHANNEL, 0);
        ledcWrite(MOTOR_R_CHANNEL, speed);
    } else if (motorLEN && (dir == CCW)) {
        ledcWrite(MOTOR_R_CHANNEL, 0);
        ledcWrite(MOTOR_L_CHANNEL, speed);
    } else {
        ledcWrite(MOTOR_R_CHANNEL, 0);
        ledcWrite(MOTOR_L_CHANNEL, 0);
    }
}
