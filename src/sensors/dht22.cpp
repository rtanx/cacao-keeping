#include "sensors/dht22.hpp"

namespace sensors {

DHT22::DHT22(uint8_t pin) {
    this->pin = pin;
    this->core = new ::DHT_Unified(pin, this->type);
}

void DHT22::begin() {
    // Initialize device.
    this->core->begin();
    Serial.println("========== DHT22 Sensor =============");
    // Print temperature sensor details.
    sensor_t sensor;
    this->core->temperature().getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.println("Temperature Sensor");
    Serial.print("Sensor Type: ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:  ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:   ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:   ");
    Serial.print(sensor.max_value);
    Serial.println("°C");
    Serial.print("Min Value:   ");
    Serial.print(sensor.min_value);
    Serial.println("°C");
    Serial.print("Resolution:  ");
    Serial.print(sensor.resolution);
    Serial.println("°C");
    Serial.println("------------------------------------");
    // Print humidity sensor details.
    this->core->humidity().getSensor(&sensor);
    Serial.println("Humidity Sensor");
    Serial.print("Sensor Type: ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:  ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:   ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:   ");
    Serial.print(sensor.max_value);
    Serial.println("%");
    Serial.print("Min Value:   ");
    Serial.print(sensor.min_value);
    Serial.println("%");
    Serial.print("Resolution:  ");
    Serial.print(sensor.resolution);
    Serial.println("%");
    Serial.println("------------------------------------");
    Serial.println("======================================");
}

void DHT22::read() {
    sensors_event_t temp_event;
    sensors_event_t hum_event;
    this->core->temperature().getEvent(&temp_event);
    this->core->humidity().getEvent(&hum_event);
    if (isnan(temp_event.temperature) || isnan(hum_event.relative_humidity)) {
        Serial.println("[ERROR] Something went wrong when reading sensor!");
        return;
    }
    this->temp_event = temp_event;
    this->hum_event = hum_event;
}

float DHT22::getTemperature() {
    return this->temp_event.temperature;
}

float DHT22::getRelativeHumidity() {
    return this->hum_event.relative_humidity;
}

}  // namespace sensors
