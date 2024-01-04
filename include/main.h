#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>

// ============= SENSOR ===================
// -------------- DHT 11 ------------------
#define DHTPIN 2
#define DHTTYPE DHT11

// ########################################

// ============= Networks =================
// ------------- WiFi ---------------------
void netsScanAndConnect();
int *netsScanAndFind(struct WifiConf *wiconf, size_t conf_size, size_t *n_nets_found);

// ============= SENSOR ===================
// -------------- DHT 11 ------------------
void setupHumidityAndTemperaturSensor();
void humidtyAndTemperatureReader();

#endif