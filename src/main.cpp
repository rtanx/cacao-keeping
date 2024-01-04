#include "main.h"

#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <stdio.h>

#include <string>

// Network Configuration
struct WifiConf {
    std::string SSID;
    std::string PWD;
};

WifiConf wifi_creds[6] = {
    {"GenerativeAI_5G", "Blackbox3.1415"},
    {"Rheza Wifi Biasa", "kontolnjepat69"},
    {"Rheza Wifi Biasa_5G", "kontolnjepat69"},
    {"TK 01 Biznet 5G", "11001101001101"},
    {"TK 01 Biznet", "kontolkugedebanget"},
    {"GenerativeAI", "Blackbox3.1415"},

};

// Sensor Configuration
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t DT11delayMS;

// Built-in device temperature
temperature_sensor_handle_t device_tem_handle = NULL;

void setup() {
    Serial.begin(115200);
    Serial.println("Setting up...");
    netsScanAndConnect();
    setupHumidityAndTemperaturSensor();
    delay(100);

    Serial.println("\nSetup done.");
}

void netsScanAndConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("Device MAC Address: ");
    Serial.print(WiFi.macAddress());
    Serial.println("");
    size_t n;
    int *nets_found = netsScanAndFind(wifi_creds, 6, &n);

    for (size_t i = 0; i < n; i++) {
        WifiConf *conf = &wifi_creds[nets_found[i]];
        Serial.printf("Trying to connect \"%s\"...\n", conf->SSID.c_str());
        WiFi.begin(conf->SSID.c_str(), conf->PWD.c_str());
        int one_sec = 1000;
        int max_wait_time = 3 * 60 * one_sec;  // 3 minute
        int wait_time = 0;
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print("#");
            delay(one_sec);
            wait_time += one_sec;
            if (wait_time >= max_wait_time) {
                Serial.println("\nRequest timeout, will trying another (if any)");
                break;
            }
            if (WiFi.status() == WL_CONNECT_FAILED) {
                Serial.printf("\nFailed to connect \"%s\", will trying another (if any)\n", conf->SSID.c_str());
                break;
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\nConnected to \"%s\" WiFi network\n", conf->SSID.c_str());
            break;
        }
    }
    heap_caps_free(nets_found);
}

int *netsScanAndFind(struct WifiConf *wiconf, size_t conf_size, size_t *n_nets_found) {
    Serial.println("Scanning network...");
    int nets_num = WiFi.scanNetworks();

    int heap_sz = 0;
    // int *nets_found = (int *)malloc(1 * sizeof(int));
    int *nets_found = (int *)heap_caps_malloc(1 * sizeof(int), MALLOC_CAP_DEFAULT);

    if (nets_found == NULL) {
        Serial.println("[ERROR] Memory allocation failed.");
        exit(EXIT_FAILURE);
        return NULL;
    }

    if (nets_num == 0) {
        Serial.println("No networks found");
    } else {
        Serial.printf("%d network found\n", nets_num);
        Serial.println("Nr | SSID\t\t\t\t| RSSI | CH | Encryption");

        for (size_t i = 0; i < nets_num; i++) {
            std::string found_ssid = WiFi.SSID(i).c_str();

            for (size_t j = 0; j < conf_size; j++) {
                if (strcmp(found_ssid.c_str(), wiconf[j].SSID.c_str()) == 0) {
                    // nets_found = (int *)realloc(nets_found, 1 * sizeof(int));
                    heap_sz += 1;
                    nets_found = (int *)heap_caps_realloc(nets_found, heap_sz * sizeof(int), MALLOC_CAP_DEFAULT);
                    nets_found[heap_sz - 1] = j;
                }
            }

            Serial.printf("%2d", i + 1);
            Serial.print(" | ");
            Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4d", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2d", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i)) {
                case WIFI_AUTH_OPEN:
                    Serial.print("open");
                    break;
                case WIFI_AUTH_WEP:
                    Serial.print("WEP");
                    break;
                case WIFI_AUTH_WPA_PSK:
                    Serial.print("WPA");
                    break;
                case WIFI_AUTH_WPA2_PSK:
                    Serial.print("WPA2");
                    break;
                case WIFI_AUTH_WPA_WPA2_PSK:
                    Serial.print("WPA+WPA2");
                    break;
                case WIFI_AUTH_WPA2_ENTERPRISE:
                    Serial.print("WPA2-EAP");
                    break;
                case WIFI_AUTH_WPA3_PSK:
                    Serial.print("WPA3");
                    break;
                case WIFI_AUTH_WPA2_WPA3_PSK:
                    Serial.print("WPA2+WPA3");
                    break;
                case WIFI_AUTH_WAPI_PSK:
                    Serial.print("WAPI");
                    break;
                default:
                    Serial.print("unknown");
            }
            Serial.println();
            delay(10);
        }
    }
    Serial.println("");

    WiFi.scanDelete();

    *n_nets_found = heap_sz;
    return nets_found;
}
void setupHumidityAndTemperaturSensor() {
    // Initialize device.
    dht.begin();
    Serial.println(F("========== DHT11 Unified Sensor Information ============="));
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
    DT11delayMS = sensor.min_delay / 1000;
}
void humidtyAndTemperatureReader() {
    // Get sensor event and print its value.
    sensors_event_t tem_event;
    sensors_event_t hum_event;

    dht.temperature().getEvent(&tem_event);
    dht.humidity().getEvent(&hum_event);
    if (isnan(tem_event.temperature) || isnan(hum_event.relative_humidity)) {
        Serial.println("[ERROR] Something went wrong when reading sensor!");
        return;
    } else {
        Serial.println("------------------------------------");
        Serial.printf("| REL HUMIDITY   | %.3f  % |\n", hum_event.relative_humidity);
        Serial.printf("| TEMPERATURE    | %.3f °C |\n", tem_event.temperature);
        Serial.println("------------------------------------");
    }
}

void loop() {
    // delay(10000);
    // Serial.printf("WiFi Status: %d\n", WiFi.status());

    // Delay between measurements.
    delay(DT11delayMS);

    humidtyAndTemperatureReader();
}