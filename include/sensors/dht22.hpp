#ifndef CACAOKEEPING_SENSORS_DHT22_H_
#define CACAOKEEPING_SENSORS_DHT22_H_

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

namespace sensors {
class DHT22 {
   private:
    ::DHT_Unified* core;
    uint8_t pin;
    const uint8_t type = 22;
    ::sensors_event_t temp_event;
    ::sensors_event_t hum_event;

   public:
    DHT22(uint8_t pin);
    void begin();
    void read();
    float getTemperature();
    float getRelativeHumidity();
};

}  // namespace sensors

#endif