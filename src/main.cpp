#include "main.h"

#include <Arduino.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include <stdio.h>

#include "actuator.h"
#include "network.h"

// DHT22 Sensor Configuration
DHT_Unified dht(DHT_SENSOR_PIN, DHTTYPE);
unsigned long sensorDelayMs;
// DS18B20 Sensor Configuration
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20TempSensor(&oneWire);
// MQ3 Alcohol Sensor Configuration
MQUnifiedsensor MQ3(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ3_SENSOR_PIN, "MQ-3");

BLYNK_WRITE(V4) {
    int val = param.asInt();
    bool bval = (bool)val;
    enableMotor(bval, bval);
    Serial.printf("Motor Switch changed: %s\n", ((bval) ? "On" : "Off"));
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
    Serial.println("========== DHT22 Sensor =============");
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
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
    dht.humidity().getSensor(&sensor);
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
    // Set delay between sensor readings based on sensor details.
    sensorDelayMs = sensor.min_delay / 1000;
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

void setupAlcoholSensor() {
    Serial.println("========== MQ3 Sensor =============");
    pinMode(MQ3_SENSOR_PIN, INPUT);  // Analog

    // Set math model to calculate the PPM concentration and the value of constants
    MQ3.setRegressionMethod(1);  // _PPM = a*ratio^b
    // Configure the equation to calculate Alcohol concentration value
    /*
     * Exponential regression:
     * Gas    | a      | b
     * H2     | 987.99 | -2.162
     * LPG    | 574.25 | -2.222
     * CO     | 36974  | -3.109
     * Alcohol| 3616.1 | -2.675
     * Propane| 658.71 | -2.168
     */
    MQ3.setA(3616.1);
    MQ3.setB(-2.675);

    MQ3.init();

    // Explanation:
    // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
    // and on clean air (Calibration conditions), setting up R0 value.
    // We recomend executing this routine only on setup in laboratory conditions.
    // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
    // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
    Serial.print("Calibrating MQ3.");
    float calcR0 = 0;
    for (size_t i = 0; i < 10; i++) {
        MQ3.update();
        calcR0 += MQ3.calibrate(RatioMQ3CleanAir);
        Serial.print(".");
    }
    MQ3.setR0(calcR0 / 10);
    Serial.println("");
    Serial.println("Calibrating Done.");

    if (isinf(calcR0)) {
        Serial.println("[WARNING]: Connection issue, R0 is inifinite (Open circuit detected), please check your wiring and supply");
        while (1)
            ;
    }
    if (calcR0 == 0) {
        Serial.println("[WARNING]: Connection issue found, R0 is zero (Analog pin shorts to ground), please check your wiring and supply");
    }

    // MQ3.serialDebug(true);  // Uncomment if you want to print the table on the serial port
}

float alcoholSensorReader() {
    MQ3.update();  // Update data, board will read the voltage from the defined analog pin
    float alcohol_ppm = MQ3.readSensor();
    // MQ3.serialDebug();
    return alcohol_ppm;
}

void sensorDataSend(bool debug_serial) {
    DHTSensorOutput dht_sensor_out;
    humidtyAndTemperatureReader(&dht_sensor_out);
    float temp = ds18b20TempSensorReader();
    float alc_ppm = alcoholSensorReader();

    if (debug_serial) {
        Serial.println("------------------------------------");
        Serial.printf("| REL HUMIDITY           | %.3f  %c  |\n", dht_sensor_out.relative_humidity, '%');
        Serial.printf("| AMBIENT TEMPERATURE    | %.3f °C   |\n", dht_sensor_out.temperature);
        Serial.printf("| TEMPERATURE            | %.3f °C   |\n", temp, '%');
        Serial.printf("| ALCOHOL                | %.3f PPM  |\n", alc_ppm);
        Serial.println("------------------------------------");
    }

    Blynk.beginGroup();
    Blynk.virtualWrite(V0, dht_sensor_out.relative_humidity);
    Blynk.virtualWrite(V1, dht_sensor_out.temperature);
    Blynk.virtualWrite(V2, temp);
    Blynk.virtualWrite(V3, alc_ppm);
    Blynk.endGroup();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setting up...");
    pinMode(LED_PIN, OUTPUT);  // Digital
    setupHumidityAndTemperaturSensor();
    setupAlcoholSensor();
    setupMotorDriver();
    ds18b20TempSensor.begin();
    delay(100);

    bool ok = beginBlynkWithWifiConnection();
    if (!ok) {
        Serial.println("[ERROR]: Failed to connect to WiFi");
    }

    blynkTimer.setInterval(sensorDelayMs, []() {
        blynkTimer.setTimeout(sensorDelayMs / 2, []() {
            digitalWrite(LED_PIN, LOW);
        });
        sensorDataSend(true);
        digitalWrite(LED_PIN, HIGH);
    });

    blynkTimer.setInterval(5, []() {
        runMotor(CW, 255);
    });

    Serial.printf("Sensor reading and data exchange to the server is carried out every %lu milliseconds", sensorDelayMs);
    Serial.println("\nSetup done.");
}

void loop() {
    Blynk.run();
    blynkTimer.run();
}
