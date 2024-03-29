#ifndef MAIN_H
#define MAIN_H

#define BLYNK_TEMPLATE_ID "TMPL6-Xb99ELJ"
#define BLYNK_TEMPLATE_NAME "CacaoKeeping"
#define BLYNK_AUTH_TOKEN "Q7KsDFOV89IgQ0zd5rmZyeh3o3nSQRxN"

#define BLYNK_PRINT Serial

#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <DHT_U.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include <string>
// ################ UTILITIES #######################
#define BOARD "ESP-32"
#define LED_PIN 5

#define VOLTAGE_RESOLUTION 3.3
#define ADC_BIT_RESOLUTION 12
// ################# BLYNK ##########################
BlynkTimer blynkTimer;

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0);

BLYNK_CONNECTED();

// ============= SENSOR ===================
// -------------- DHT22 -------------------
#define DHT_SENSOR_PIN 4
#define DHTTYPE DHT22
struct DHTSensorOutput {
    float relative_humidity;
    float temperature;
};
// ----- DS18B20 Temperature Sensor -------
#define ONE_WIRE_BUS 2
// ------- MQ3 Alcohol Sensor -------------
#define MQ3_SENSOR_PIN A3
#define RatioMQ3CleanAir 9.83  // RS / R0 = 9.83 ppm

// ============= Networks =================
// ------------- WiFi ---------------------
bool beginBlynkWithWifiConnection();
// ########################################

// ============= SENSOR ===================
void sensorDataSend(bool debug_serial);
// -------------- DHT 22 ------------------
void setupHumidityAndTemperaturSensor();
void humidtyAndTemperatureReader(DHTSensorOutput *out);
// ----- DS18B20 Temperature Sensor -------
float ds18b20TempSensorReader();
// ------- MQ3 Alcohol Sensor -------------
void setupAlcoholSensor();
float alcoholSensorReader();

#endif