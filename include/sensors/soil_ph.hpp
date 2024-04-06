#ifndef CACAOKEEPING_SENSORS_SOILPH_H_
#define CACAOKEEPING_SENSORS_SOILPH_H_
#include <Arduino.h>

namespace sensors {

class SoilPH {
   private:
    uint8_t pin;
    float a;
    float b;

   public:
    SoilPH(uint8_t pin, float a, float b);
    void begin();
    float read();
};

}  // namespace sensors

#endif