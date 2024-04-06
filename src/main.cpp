#include "main.h"

#include <Arduino.h>
// #include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <Wire.h>
#include <stdio.h>
#include <strings.h>

#include "actuator.h"
#include "actuators/bts7960_motor_driver.hpp"
#include "network.h"
#include "sensors/dht22.hpp"
#include "sensors/ds18b20.hpp"
#include "sensors/mq3.hpp"
#include "sensors/soil_ph.hpp"

const unsigned long motorAutoStirInterval = 1000 * 60 * 60 * 24 * 2;  // 2 days
const unsigned long motorAutoStirTimeout = 1000 * 60 * 2;             // 2 minutes
const unsigned long sensingDelayMs = 2000;

bool isAutoStirRunning = false;

LiquidCrystal_I2C lcd(0x27, 20, 4);
// DHT22 Sensor Configuration
sensors::DHT22 DHT22Sensor(DHT_SENSOR_PIN);
// DS18B20 Sensor Configuration
OneWire oneWire(ONE_WIRE_BUS);
sensors::DS18B20Temperature DS18B20Sensor(&oneWire);
// MQ3 Alcohol Sensor Configuration
sensors::MQ3 MQ3Sensor(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ3_SENSOR_PIN);
// Soil PH Sensor Configuration
sensors::SoilPH SoilPHSensor(SOIL_PH_SENSOR_PIN, -0.00668, 12.3);
// BTS7960 Actuator Configuration
actuators::BTS7960MotorDriver MotorDriver(MOTOR_LPWM_PIN, MOTOR_RPWM_PIN, MOTOR_LEN_PIN, MOTOR_REN_PIN, MOTOR_POWER_SWITCH_PIN, MOTOR_L_CHANNEL, MOTOR_R_CHANNEL, MOTOR_FREQUENCY, MOTOR_RESOLUTION);

BLYNK_WRITE(V4) {
    int val = param.asInt();
    bool bval = (bool)val;
    if (isAutoStirRunning) {
        Blynk.virtualWrite(V4, true);
        return;
    }

    if (bval) {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Pengadukan sedang");
        lcd.setCursor(4, 2);
        lcd.print("berjalan...");

        MotorDriver.turnCW();
    } else {
        MotorDriver.stop();
        lcd.clear();
    }

    Serial.printf("Motor Switch changed: %s\n", ((bval) ? "On" : "Off"));
    // SerialBT.printf("Motor Switch changed: %s\n", ((bval) ? "On" : "Off"));
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

void sensorDataSend(bool debug_serial) {
    float temp = DS18B20Sensor.read();
    float alc_ppm = MQ3Sensor.read();

    DHT22Sensor.read();
    float relative_hum = DHT22Sensor.getRelativeHumidity();
    float ambient_temp = DHT22Sensor.getTemperature();

    if (debug_serial) {
        Serial.println("------------------------------------");
        Serial.printf("| REL HUMIDITY           | %.3f  %c  |\n", relative_hum, '%');
        Serial.printf("| AMBIENT TEMPERATURE    | %.3f °C   |\n", ambient_temp);
        Serial.printf("| TEMPERATURE            | %.3f °C   |\n", temp);
        Serial.printf("| ALCOHOL                | %.3f PPM  |\n", alc_ppm);
        Serial.println("------------------------------------");
    }
    if (!MotorDriver.isRunning()) {
        lcd.setCursor(0, 0);
        lcd.printf("Kelembapan: %.1f %c", relative_hum, '%');
        lcd.setCursor(0, 1);
        lcd.printf("Suhu Box : %.1f C", ambient_temp);
        lcd.setCursor(0, 2);
        lcd.printf("Suhu Kakao : %.1f C", temp);
        lcd.setCursor(0, 3);
        lcd.printf("Alkohol : %.3f ppm", alc_ppm);
    }

    Blynk.beginGroup();
    Blynk.virtualWrite(V0, relative_hum);
    Blynk.virtualWrite(V1, ambient_temp);
    Blynk.virtualWrite(V2, temp);
    Blynk.virtualWrite(V3, alc_ppm);
    Blynk.endGroup();
}

void setup() {
    Serial.begin(115200);
    // SerialBT.begin("ESP32 Bluetooth");
    Serial.println("Setting up...");
    scanI2CWire();
    delay(300);
    lcd.init();
    lcd.backlight();
    lcd.noBlink();
    delay(500);
    lcd.clear();
    lcd.setCursor(6, 1);
    lcd.print("Memulai...");
    pinMode(LED_PIN, OUTPUT);  // Digital

    DHT22Sensor.begin();
    MQ3Sensor.begin();
    DS18B20Sensor.begin();
    SoilPHSensor.begin();

    MotorDriver.begin();

    MotorDriver.startPower();
    MotorDriver.enableCCW();
    MotorDriver.enableCW();
    MotorDriver.setSpeed(255);

    delay(100);
    bool ok = beginBlynkWithWifiConnection();
    if (!ok) {
        Serial.println("[ERROR]: Failed to connect to WiFi");
    }

    blynkTimer.setInterval(sensingDelayMs, []() {
        blynkTimer.setTimeout(sensingDelayMs / 2, []() {
            digitalWrite(LED_PIN, LOW);
        });
        sensorDataSend(true);
        digitalWrite(LED_PIN, HIGH);
    });

    blynkTimer.setInterval(motorAutoStirInterval, []() {
        blynkTimer.setTimeout(motorAutoStirTimeout, []() {
            MotorDriver.stop();
            isAutoStirRunning = false;
            Blynk.virtualWrite(V4, false);
            lcd.clear();
        });
        Blynk.virtualWrite(V4, true);
        MotorDriver.turnCW();
        isAutoStirRunning = true;
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Pengadukan otomatis");
        lcd.setCursor(1, 2);
        lcd.print("sedang berjalan...");
    });

    lcd.clear();
    Serial.printf("Sensor reading and data exchange to the server is carried out every %lu milliseconds", sensingDelayMs);
    Serial.println("\nSetup done.");
}

void scanI2CWire() {
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    Wire.begin(GPIO_NUM_22, GPIO_NUM_21);
    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
            delay(1);
        }
    }
    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");
}

void loop() {
    Blynk.run();
    blynkTimer.run();
}
