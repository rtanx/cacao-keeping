#include <sensors/soil_ph.hpp>

namespace sensors {

SoilPH::SoilPH(uint8_t pin, float a, float b) {
    this->pin = pin;
    this->a = a;
    this->b = b;
}

void SoilPH::begin() {
    pinMode(this->pin, INPUT);
}

float SoilPH::read() {
    uint16_t analog_val = analogRead(this->pin);
    Serial.print("analog val: ");
    Serial.println(analog_val);
    float a = this->a;
    float b = this->b;
    return (a * analog_val) + b;
}

}  // namespace sensors
