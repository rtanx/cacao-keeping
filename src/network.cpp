#include "network.h"

#include <WiFi.h>
#include <string.h>

#include <nlohmann/json.hpp>

using namespace std;

WifiConf *connected_net = nullptr;

WifiConf wifi_creds[7] = {
    // Initial wifi configuration (wifi credentials here just for local run purpose)
    {"HiggsBoson_5G", "Blackbox3.1415"},
    {"HiggsBoson", "Blackbox3.1415"},
    {"dummy", "dummy"},
    {"dummy1", "dummy1"},
    {"dummy2", "dummy3"},
    {"dummy3", "dummy3"},
    {"dummy4", "dummy4"},
};

// DEPRECATED
bool netsScanAndConnect(WifiConf *connected_net) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    size_t n;
    int *nets_found = netsScanAndFind(wifi_creds, 6, &n);
    bool is_connected = false;
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
            is_connected = true;
            *connected_net = *conf;
            break;
        }
    }
    heap_caps_free(nets_found);
    return is_connected;
}
bool getWiFiCredentialsByIndex(size_t idx, WifiConf *out) {
    if (idx > 7) {
        return false;
    }
    *out = wifi_creds[idx];
    return true;
}
int *netScanAndMatchCreds(size_t *n_nets_found) {
    return netsScanAndFind(wifi_creds, 7, n_nets_found);
}
int *netsScanAndFind(WifiConf *wiconf, size_t conf_size, size_t *n_nets_found) {
    Serial.print("Device MAC Address: ");
    Serial.print(WiFi.macAddress());
    Serial.println("");

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