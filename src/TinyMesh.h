#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <LoRa.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <mbedtls/aes.h>
#include "TinyMeshConfig.h"

class TinyMeshClass {
public:
    TinyMeshClass();
    bool begin();
    void update();
    void setBLEPasscode(int passcode);
    void setNodeName(const char* name);
    void setDebugEnable(bool enable);
    void setDebugLevel(uint8_t level);
    void processCommand(const String& input);
    void debugLog(uint8_t level, const char* message);
    
    uint32_t nodeId;
    bool deviceConnected;
    unsigned long lastActivity;
    uint32_t messageCounter;
    
private:
    struct Node {
        uint32_t id;
        char name[16];
        int8_t lastSNR;
        int rssi;
        uint32_t lastHeard;
        uint8_t batteryLevel;
        uint8_t hops;
        bool isOnline;
        float distance;
    } routingTable[TINYMESH_MAX_NODES];

    struct Channel {
        uint8_t id;
        char name[16];
        bool enabled;
        byte encKey[16];
        uint32_t frequency;
        uint8_t bandwidth;
        uint8_t spreadFactor;
        uint8_t codingRate;
    } channels[TINYMESH_MAX_CHANNELS];

    struct TinyMeshPacket {
        uint32_t to;
        uint32_t from;
        uint8_t type;
        uint8_t channel;
        uint8_t hops;
        uint8_t flags;
        uint32_t id;
        uint8_t payload[240];
        uint8_t payloadLen;
    };

    char nodeName[16];
    Preferences preferences;
    BLEServer* pServer;
    BLECharacteristic* pCharacteristic;
    unsigned long lastDiscovery;
    unsigned long lastPositionUpdate;
    int blePasscode;
    bool powerSavingEnabled;
    bool debugEnabled;
    uint8_t debugLevel;

    void initializeChannels();
    void loadChannelsFromPreferences();
    void saveChannelToPreferences(uint8_t channelNum);
    bool initLoRa();
    bool initBLE();
    void clearRoutingTable();
    void updateRoutingTable(uint32_t id, const char* name, int rssi, int8_t snr, uint8_t hops);
    void updateNodeStatus();
    void processPacket(TinyMeshPacket* packet, int rssi, float snr);
    void sendResponse(JsonDocument& doc);
    void sendResponse(const char* jsonStr);
    bool sendPacket(const TinyMeshPacket* packet);
    void receivePackets();
    void sendDiscoveryPacket();
    void sendPositionUpdate();
    void managePower();
    uint32_t generateNodeId();
    float calculateDistance(int rssi);
    
    friend class MyServerCallbacks;
    friend class MyCharacteristicCallbacks;
};

extern TinyMeshClass TinyMesh;