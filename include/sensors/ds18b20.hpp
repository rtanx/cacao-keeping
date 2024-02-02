#ifndef CACAOKEEPING_SENSORS_DS18B20_H_
#define CACAOKEEPING_SENSORS_DS18B20_H_

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

namespace sensors {

class DS18B20Temperature {
   private:
    ::DallasTemperature core;
    ::DeviceAddress device_address;
    ::OneWire* one_wire;

    uint8_t resolution;

   public:
    DS18B20Temperature(::OneWire* oneWire, uint8_t resolution = 9);
    DS18B20Temperature(uint8_t one_wire_pin, uint8_t resolution = 9);
    ~DS18B20Temperature();
    ::OneWire* getOneWireInstance();
    void begin();
    float read();
};

}  // namespace sensors

#endif