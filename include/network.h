#ifndef NETWORK_H
#define NETWORK_H

#include <WiFi.h>

#include <string>

struct WifiConf {
    std::string SSID;
    std::string PWD;
};

bool netsScanAndConnect(WifiConf *connected_net);
int *netsScanAndFind(WifiConf *wiconf, size_t conf_size, size_t *n_nets_found);
bool getWiFiCredentialsByIndex(size_t idx, WifiConf *out);
int *netScanAndMatchCreds(size_t *n_nets_found);

#endif