#ifndef MAIN_H
#define MAIN_H

#include "secret.h"

#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <BlynkSimpleEsp32.h>

#include <string>
// ################ UTILITIES #######################
#define BOARD "ESP-32"
#define USE_ESP32_DEV_MODULE
#define LED_PIN GPIO_NUM_23

#define VOLTAGE_RESOLUTION 3.3
#define ADC_BIT_RESOLUTION 12

// ======================================== SENSOR ========================================
// -------------- DHT22 -------------------
#define DHT_SENSOR_PIN GPIO_NUM_4

// ----- DS18B20 Temperature Sensor -------
#define ONE_WIRE_BUS 2

// ------- MQ3 Alcohol Sensor -------------
#define MQ3_SENSOR_PIN GPIO_NUM_39

// ------- Soil PH Sensor -------------
#define SOIL_PH_SENSOR_PIN GPIO_NUM_27

// ======================================== ACTUATOR ========================================
#define MOTOR_LPWM_PIN GPIO_NUM_33
#define MOTOR_RPWM_PIN GPIO_NUM_32
#define MOTOR_LEN_PIN GPIO_NUM_26
#define MOTOR_REN_PIN GPIO_NUM_25
#define MOTOR_POWER_SWITCH_PIN GPIO_NUM_5

#define MOTOR_L_CHANNEL 15
#define MOTOR_R_CHANNEL 14

#define MOTOR_FREQUENCY 1000
#define MOTOR_RESOLUTION 8

// ======================================== Networks ========================================
// ------------- Blynk ---------------------
BlynkTimer blynkTimer;

// ======================================== MAIN ========================================
bool beginBlynkWithWifiConnection();
void sensorDataSend(bool debug_serial);
void scanI2CWire();
#endif