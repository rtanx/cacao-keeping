#include "sensors/ds18b20.hpp"

namespace sensors {

DS18B20Temperature::DS18B20Temperature(::OneWire* one_wire, uint8_t resolution /* = 9 */) {
    this->resolution = resolution;
    this->one_wire = one_wire;
    this->core = DallasTemperature(one_wire);
}

DS18B20Temperature::DS18B20Temperature(uint8_t one_wire_pin, uint8_t resolution /* = 9 */) {
    this->resolution = resolution;
    *this->one_wire = OneWire(one_wire_pin);
    this->core = DallasTemperature(this->one_wire);
}

DS18B20Temperature::~DS18B20Temperature() {
}

::OneWire* DS18B20Temperature::getOneWireInstance() {
    return this->one_wire;
}

void DS18B20Temperature::begin() {
    Serial.println("======================== DS18B20 Temperature Sensor ========================");
    Serial.println("Locating devices...");
    this->core.begin();
    Serial.print("Found ");
    Serial.print(this->core.getDeviceCount(), DEC);
    Serial.println(" device(s).");

    Serial.println("Parasite power is: ");
    Serial.println(this->core.isParasitePowerMode() ? "ON" : "OFF");

    if (!this->core.getAddress(this->device_address, 0)) {
        Serial.println("Unable to find address for device 0");
    }

    Serial.print("Device 0 address: ");
    for (uint8_t i = 0; i < 8; i++) {
        if (device_address[i] < 16) {
            Serial.print("0");
        }
        Serial.print(device_address[i], HEX);
    }
    Serial.println();

    this->core.setResolution(this->device_address, this->resolution);

    Serial.print("Device 0 resolution: ");
    Serial.print(this->core.getResolution(this->device_address), DEC);
    Serial.println();
    Serial.println("============================================================================");
}

float DS18B20Temperature::read() {
    this->core.requestTemperatures();
    float temp_c = this->core.getTempC(this->device_address);
    if (temp_c == DEVICE_DISCONNECTED_C) {
        Serial.println("[ERROR]: Could not read temperature data from DS1820 Sensor, device disconnected.");
    }
    return temp_c;
}

}  // namespace sensors
