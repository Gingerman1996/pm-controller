#include "ESPNowMaster.h"

// Define static members
ESPNow_message ESPNowMaster::boardsStruct[128] = {};
ESPNowMaster* ESPNowMaster::instance = nullptr; // Initialize the static instance pointer

ESPNowMaster::ESPNowMaster(const char* ssid, const char* password, const char* apSSID)
    : _ssid(ssid), _password(password), _apSSID(apSSID), timeClient(ntpUDP, "pool.ntp.org", 0, 60000) {
    instance = this; // Assign current instance to the static instance pointer
}

void ESPNowMaster::begin() {
    Serial.begin(115200);

    // Connect to WiFi router
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Start NTP client
    timeClient.begin();
    while (!timeClient.update()) {
        timeClient.forceUpdate();
        delay(1000);
        Serial.println("Updating NTP time...");
    }
    Serial.printf("NTP time synchronized: %lu\n", timeClient.getEpochTime());

    // Set up as Access Point
    WiFi.softAP(_apSSID);

    // Ensure AP and STA are on the same channel
    int staChannel = WiFi.channel();
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(_apSSID, NULL, staChannel);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
}

void ESPNowMaster::onDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
    if (instance == nullptr) {
        Serial.println("No instance available to handle the received data.");
        return;
    }

    char macStr[18];
    // Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    // Serial.println(macStr);

    ESPNow_message ESPNowData;
    memcpy(&ESPNowData, incomingData, sizeof(ESPNowData));

    // Use instance to access timeClient
    ESPNowData.timestamp = instance->timeClient.getEpochTime();
    // Serial.printf("Board ID (dec): %llu, Board ID (hex): %llX: %d bytes\n", ESPNowData.id, ESPNowData.id, len);

    bool found = false;

    // Loop through the array to find an empty slot or a matching ID
    for (int i = 0; i < 128; i++) {
        if (boardsStruct[i].id == 0) {
            boardsStruct[i] = ESPNowData;
            // Serial.printf("New board added at index %d with ID (hex): %llX\n", i, ESPNowData.id);
            found = true;
            break;
        } else if (boardsStruct[i].id == ESPNowData.id) {
            boardsStruct[i] = ESPNowData;
            // Serial.printf("Updated board at index %d with ID (hex): %llX\n", i, ESPNowData.id);
            found = true;
            break;
        }
    }

    if (!found) {
        Serial.println("No available slot or matching ID to store data.");
    }
}

bool ESPNowMaster::getData(uint64_t boardId, float &pm25, float &temp, float &humi, unsigned long &timestamp) {
    for (int i = 0; i < 128; i++) {
        if (boardsStruct[i].id == boardId) {
            pm25 = boardsStruct[i].pm25;
            temp = boardsStruct[i].temp;
            humi = boardsStruct[i].humi;
            timestamp = boardsStruct[i].timestamp;
            return true;
        }
    }
    return false;
}

bool ESPNowMaster::getBoardId(int index, uint64_t &boardId) {
    if (index >= 0 && index < 128 && boardsStruct[index].id != 0) {
        boardId = boardsStruct[index].id;
        return true;
    }
    return false;
}

void ESPNowMaster::checkBoardTimeout() {
    instance->timeClient.update();
    unsigned long currentTimestamp = instance->timeClient.getEpochTime();
    for (int i = 0; i < 128; i++) {
        if (boardsStruct[i].id != 0 && (currentTimestamp - boardsStruct[i].timestamp) > 30) {
            Serial.printf("Removing board at index %d with ID (hex): %llX due to timeout\n", i, boardsStruct[i].id);
            boardsStruct[i].id = 0;
        }
    }
}
