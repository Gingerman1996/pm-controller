#ifndef ESPNOWMASTER_H
#define ESPNOWMASTER_H

#include <WiFi.h>
#include <esp_now.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "MyLog.h"

struct ESPNow_message {
    uint64_t id;
    float pm25;
    float humi;
    float temp;
    unsigned long timestamp;
};

class ESPNowMaster {
public:
    ESPNowMaster(const char* ssid, const char* password, const char* _apSSID);
    void begin();
    bool getData(uint64_t boardId, float &pm25, float &temp, float &humi, unsigned long &timestamp);
    bool getBoardId(int index, uint64_t &boardId);
    void checkBoardTimeout();

private:
    static void onDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len);
    static ESPNow_message boardsStruct[128];
    static ESPNowMaster* instance;  // Static instance pointer

    WiFiUDP ntpUDP;
    NTPClient timeClient;
    const char* _ssid;
    const char* _password;
    const char* _apSSID;
};

#endif
