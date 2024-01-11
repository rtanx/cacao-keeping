#include "main.h"

#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <stdio.h>

#include "network.h"

// DHT22 Sensor Configuration
DHT_Unified dht(DHT_SENSOR_PIN, DHTTYPE);
uint32_t DHTSensordelayMS;
// DS18B20 Sensor Configuration
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20TempSensor(&oneWire);

BLYNK_WRITE(V0) {
}

BLYNK_CONNECTED() {
    uint8_t led_state = HIGH;
    blynkTimer.setTimer(
        250L, [led_state]() mutable {
            digitalWrite(LED_PIN, led_state);
            led_state = !led_state;
        },
        6);
}
bool beginBlynkWithWifiConnection() {
    WiFi.mode(WIFI_STA);

    WiFi.disconnect();

    size_t n;
    int *nets_found = netScanAndMatchCreds(&n);
    bool is_connected = false;
    for (size_t i = 0; i < n; i++) {
        WifiConf conf;
        getWiFiCredentialsByIndex(nets_found[i], &conf);
        Serial.printf("Trying to connect \"%s\"...\n", conf.SSID.c_str());
        Blynk.begin(BLYNK_AUTH_TOKEN, conf.SSID.c_str(), conf.PWD.c_str());
        if (Blynk.connected()) {
            Serial.printf("\nConnected to \"%s\" WiFi network\n", conf.SSID.c_str());
            is_connected = true;
            break;
        } else {
            Serial.printf("\nFailed to connect \"%s\", will trying another (if any)\n", conf.SSID.c_str());
        }
    }
    heap_caps_free(nets_found);
    return is_connected;
}
void setupHumidityAndTemperaturSensor() {
    // Initialize device.
    dht.begin();
    Serial.println(F("========== DHT22 Unified Sensor Information ============="));
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("°C"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("°C"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("%"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("%"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
    // Set delay between sensor readings based on sensor details.
    DHTSensordelayMS = sensor.min_delay / 1000;
}
void humidtyAndTemperatureReader(DHTSensorOutput *out) {
    // Get sensor event and print its value.
    sensors_event_t tem_event;
    sensors_event_t hum_event;

    dht.temperature().getEvent(&tem_event);
    dht.humidity().getEvent(&hum_event);
    if (isnan(tem_event.temperature) || isnan(hum_event.relative_humidity)) {
        Serial.println("[ERROR] Something went wrong when reading sensor!");
        return;
    }
    out->relative_humidity = hum_event.relative_humidity;
    out->temperature = tem_event.temperature;
}

float ds18b20TempSensorReader() {
    ds18b20TempSensor.requestTemperatures();
    float temp_c = ds18b20TempSensor.getTempCByIndex(0);
    if (temp_c == DEVICE_DISCONNECTED_C) {
        Serial.println("[ERROR]: Could not read temperature data from DS1820 Sensor");
    }
    return temp_c;
}

float alcoholSensorReader() {
    uint16_t analog_val = analogRead(MQ3_SENSOR_PIN);
}

void sensorDataSend() {
    DHTSensorOutput dht_sensor_out;
    humidtyAndTemperatureReader(&dht_sensor_out);
    float temp = ds18b20TempSensorReader();

    Serial.println("------------------------------------");
    Serial.printf("| REL HUMIDITY           | %.3f  %c |\n", dht_sensor_out.relative_humidity, '%');
    Serial.printf("| AMBIENT TEMPERATURE    | %.3f °C |\n", dht_sensor_out.temperature);
    Serial.printf("| TEMPERATURE            | %.3f °C |\n", temp, '%');
    Serial.println("------------------------------------");

    Blynk.beginGroup();
    Blynk.virtualWrite(V0, dht_sensor_out.relative_humidity);
    Blynk.virtualWrite(V1, dht_sensor_out.temperature);
    Blynk.virtualWrite(V2, temp);
    Blynk.endGroup();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setting up...");
    pinMode(LED_PIN, OUTPUT);        // Digital
    pinMode(MQ3_SENSOR_PIN, INPUT);  // Analog
    setupHumidityAndTemperaturSensor();
    ds18b20TempSensor.begin();
    delay(100);

    bool ok = beginBlynkWithWifiConnection();
    if (!ok) {
        Serial.println("[ERROR]: Failed to connect to WiFi");
    }

    blynkTimer.setInterval((long)DHTSensordelayMS, sensorDataSend);

    Serial.println("\nSetup done.");
}

void loop() {
    Blynk.run();
    blynkTimer.run();
}
