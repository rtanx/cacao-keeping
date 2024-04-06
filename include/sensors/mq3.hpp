#ifndef CACAOKEEPING_SENSORS_MQ3_H_
#define CACAOKEEPING_SENSORS_MQ3_H_

#include <Arduino.h>
#include <MQUnifiedsensor.h>

namespace sensors {

class MQ3 {
   private:
    ::MQUnifiedsensor* core;
    const int ratio_clean_air = 60;  // RS / R0 = 60 ppm
    const String mq_type = "MQ-3";
    uint8_t pin;
    /*
     * Exponential regression:
     * Gas    | a      | b
     * LPG    | 44771  | -3.245
     * CH4    | 2*10^31| 19.01
     * CO     | 521853 | -3.821
     * Alcohol| 0.3934 | -1.504
     * Benzene| 4.8387 | -2.68
     * Hexane | 7585.3 | -2.849
     */
    const float a_regression_coeff = 0.3934;
    const float b_regression_coeff = -1.504;
    const float RL = 200;  // kOhm

    float calibrate();

   public:
    MQ3(String board, float voltage_resolution, uint8_t adc_bit_resolution, uint8_t pin);
    // ~MQ3();
    void begin();
    float read();
};

}  // namespace sensors

#endif