#include "actuators/bts7960_motor_driver.hpp"

#include <Arduino.h>

#include <algorithm>

namespace actuators {

BTS7960MotorDriver::BTS7960MotorDriver(uint8_t lpwm_pin, uint8_t rpwm_pin,
                                       uint8_t len_pin,
                                       uint8_t ren_pin,
                                       uint8_t l_channel /* = 15 */,
                                       uint8_t r_channel /* = 14 */,
                                       unsigned int frequency /* = 1000 */,
                                       uint8_t resolution /* = 8 */) {
    this->lpwm_pin = lpwm_pin;
    this->rpwm_pin = rpwm_pin;
    this->len_pin = len_pin;
    this->ren_pin = ren_pin;
    this->l_channel = l_channel;
    this->r_channel = r_channel;
    this->frequency = frequency;
    this->resolution = resolution;

    this->max_duty_cycle = (1 << resolution) - 1;
}

BTS7960MotorDriver::~BTS7960MotorDriver() {
}

void BTS7960MotorDriver::begin() {
    ledcSetup(r_channel, frequency, resolution);
    ledcSetup(r_channel, frequency, resolution);

    ledcAttachPin(rpwm_pin, r_channel);
    ledcAttachPin(lpwm_pin, l_channel);

    pinMode(len_pin, OUTPUT);
    pinMode(ren_pin, OUTPUT);
}

void BTS7960MotorDriver::enableCW() {
    this->r_enable = true;
}

void BTS7960MotorDriver::disableCW() {
    if (this->isCWEnable()) {
        ledcWrite(r_channel, 0);
        this->r_enable = false;
    }
}

void BTS7960MotorDriver::enableCCW() {
    this->l_enable = true;
}

void BTS7960MotorDriver::disableCCW() {
    if (this->isCCWEnable()) {
        ledcWrite(l_channel, 0);
        this->l_enable = false;
    }
}

bool BTS7960MotorDriver::isCWEnable() {
    return this->r_enable;
}

bool BTS7960MotorDriver::isCCWEnable() {
    return this->l_enable;
}

void BTS7960MotorDriver::turnCW(bool force /* = false */) {
    this->turn(1, force);
}

void BTS7960MotorDriver::turnCCW(bool force /* = false */) {
    this->turn(-1, force);
}

void BTS7960MotorDriver::setSpeed(uint s) {
    this->speed = std::min(s, max_duty_cycle);
}

}  // namespace actuators
