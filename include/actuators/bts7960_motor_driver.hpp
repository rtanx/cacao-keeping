#ifndef CACAOKEEPING_ACTUATORS_BTS7960MOTORDRIVER_H_
#define CACAOKEEPING_ACTUATORS_BTS7960MOTORDRIVER_H_

#include <Arduino.h>

namespace actuators {
class BTS7960MotorDriver {
   private:
    uint8_t lpwm_pin;
    uint8_t rpwm_pin;
    uint8_t len_pin;
    uint8_t ren_pin;

    uint8_t l_channel;
    uint8_t r_channel;

    uint8_t resolution;
    unsigned int frequency;

    bool l_enable = false;
    bool r_enable = false;
    uint max_duty_cycle;
    uint speed = 0;

    /**
     * @brief Turn motor to given direction
     *
     * @param[in] direction if direction < 0 it will be turned to left,
     *      if direction > 0 it will be turned to right,
     *      if direction = 0 it will be stop the motor.
     * @param[in] force force motor to turn even when the direction disabled,
     *      it will first enable the direction then turn to that direction
     */
    void turn(int8_t direction, bool force);

   public:
    BTS7960MotorDriver(uint8_t lpwm_pin,
                       uint8_t rpwm_pin,
                       uint8_t len_pin,
                       uint8_t ren_pin,
                       uint8_t l_channel = 15,
                       uint8_t r_channel = 14,
                       unsigned int frequency = 1000,
                       uint8_t resolution = 8);
    ~BTS7960MotorDriver();

    void begin();
    void enableCW();
    void disableCW();
    void enableCCW();
    void disableCCW();
    bool isCWEnable();
    bool isCCWEnable();
    void turnCW(bool force = false);
    void turnCCW(bool force = false);
    void setSpeed(uint s);
};

}  // namespace actuators

#endif