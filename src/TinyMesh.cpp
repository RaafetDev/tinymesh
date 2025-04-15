#include "TinyMesh.h"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        TinyMesh.deviceConnected = true;
        TinyMesh.lastActivity = millis();
        TinyMesh.debugLog(2, "BLE Client Connected");
    }
    
    void onDisconnect(BLEServer* pServer) override {
        TinyMesh.deviceConnected = false;
        TinyMesh.debugLog(2, "BLE Client Disconnected");
        pServer->startAdvertising();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            TinyMesh.processCommand(value);
            TinyMesh.lastActivity = millis();
        }
    }
};

TinyMeshClass::TinyMeshClass() {
    nodeId = 0;
    memset(nodeName, 0, sizeof(nodeName));
    powerSavingEnabled = TINYMESH_POWER_SAVING_DEFAULT;
    debugEnabled = TINYMESH_DEBUG_ENABLE;
    debugLevel = TINYMESH_DEBUG_LEVEL;
    lastActivity = 0;
    lastDiscovery = 0;
    lastPositionUpdate = 0;
    messageCounter = 0;
    deviceConnected = false;
    pServer = nullptr;
    pCharacteristic = nullptr;
    blePasscode = BLE_PASSCODE_DEFAULT;
}

bool TinyMeshClass::begin() {
    Serial.begin(115200);
    
    SPI.begin(TINYMESH_SCK_PIN, TINYMESH_MISO_PIN, TINYMESH_MOSI_PIN, TINYMESH_SS_PIN);
    preferences.begin("tinymesh", false);
    
    nodeId = generateNodeId();
    
    if (strlen(nodeName) == 0) {
        snprintf(nodeName, sizeof(nodeName), "TinyMesh-%05lX", nodeId & 0xFFFFF);
    }
    
    initializeChannels();
    
    if (!initLoRa()) {
        debugLog(1, "LoRa initialization failed");
        return false;
    }
    
    if (!initBLE()) {
        debugLog(1, "BLE initialization failed");
        return false;
    }
    
    clearRoutingTable();
    lastActivity = millis();
    debugLog(2, "TinyMesh initialized successfully");
    
    sendDiscoveryPacket();
    
    return true;
}

void TinyMeshClass::update() {
    unsigned long currentTime = millis();
    
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        processCommand(input);
    }
    
    receivePackets();
    
    if (currentTime - lastDiscovery > TINYMESH_DISCOVERY_INTERVAL) {
        sendDiscoveryPacket();
        lastDiscovery = currentTime;
    }
    
    if (currentTime - lastPositionUpdate > TINYMESH_POSITION_UPDATE_INTERVAL) {
        sendPositionUpdate();
        lastPositionUpdate = currentTime;
    }
    
    updateNodeStatus();
    
    if (powerSavingEnabled) {
        managePower();
    }
}

bool TinyMeshClass::initLoRa() {
    LoRa.setPins(TINYMESH_SS_PIN, TINYMESH_RST_PIN, TINYMESH_DIO_PIN);
    if (!LoRa.begin(TINYMESH_DEFAULT_FREQUENCY)) {
        return false;
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    return true;
}

bool TinyMeshClass::initBLE() {
    BLEDevice::init(nodeName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setStaticPIN(blePasscode);
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
    
    pService->start();
    pServer->getAdvertising()->start();
    return true;
}

void TinyMeshClass::setBLEPasscode(int passcode) {
    blePasscode = passcode;
    preferences.putInt("blePasscode", passcode);
    if (pServer) {
        BLEDevice::deinit();
        initBLE();
    }
}

void TinyMeshClass::setNodeName(const char* name) {
    strncpy(nodeName, name, 15);
    nodeName[15] = '\0';
    preferences.putString("nodeName", nodeName);
    if (pServer) {
        BLEDevice::deinit();
        initBLE();
    }
}

void TinyMeshClass::setDebugEnable(bool enable) {
    debugEnabled = enable;
    preferences.putBool("debugEnable", enable);
}

void TinyMeshClass::setDebugLevel(uint8_t level) {
    debugLevel = level;
    preferences.putUInt("debugLevel", level);
}

void TinyMeshClass::processCommand(const String& input) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, input);
    
    if (error) {
        sendResponse("{\"error\":\"Invalid JSON\"}");
        return;
    }
    
    const char* cmd = doc["command"];
    if (!cmd) {
        sendResponse("{\"error\":\"Missing command\"}");
        return;
    }
    
    StaticJsonDocument<512> response;
    
    if (strcmp(cmd, "MSG") == 0) {
        if (!doc.containsKey("to") || !doc.containsKey("message")) {
            sendResponse("{\"error\":\"Missing required fields\"}");
            return;
        }
        
        uint32_t target = strtoul(doc["to"], NULL, 16);
        const char* msg = doc["message"];
        uint8_t channel = doc["channel"] | 0;
        
        TinyMeshPacket packet;
        packet.to = target;
        packet.from = nodeId;
        packet.type = MSG_TYPE_DATA;
        packet.channel = channel;
        packet.hops = 0;
        packet.flags = 0x01;
        packet.id = messageCounter++;
        memcpy(packet.payload, msg, strlen(msg) + 1);
        packet.payloadLen = strlen(msg) + 1;
        
        bool success = sendPacket(&packet);
        
        response["status"] = success ? "sent" : "failed";
        response["msgId"] = packet.id;
    }
    else if (strcmp(cmd, "LIST") == 0) {
        JsonArray nodes = response.createNestedArray("nodes");
        for (int i = 0; i < TINYMESH_MAX_NODES; i++) {
            if (routingTable[i].id) {
                JsonObject node = nodes.createNestedObject();
                node["id"] = String(routingTable[i].id, HEX);
                node["name"] = routingTable[i].name;
                node["rssi"] = routingTable[i].rssi;
                node["snr"] = routingTable[i].lastSNR;
                node["distance"] = routingTable[i].distance;
                node["online"] = routingTable[i].isOnline;
                node["battery"] = routingTable[i].batteryLevel;
            }
        }
    }
    else if (strcmp(cmd, "CHANNELS") == 0) {
        JsonArray chans = response.createNestedArray("channels");
        for (int i = 0; i < TINYMESH_MAX_CHANNELS; i++) {
            if (channels[i].enabled) {
                JsonObject chan = chans.createNestedObject();
                chan["id"] = channels[i].id;
                chan["name"] = channels[i].name;
                chan["frequency"] = channels[i].frequency;
                chan["bandwidth"] = channels[i].bandwidth;
                chan["spreadFactor"] = channels[i].spreadFactor;
                chan["codingRate"] = channels[i].codingRate;
            }
        }
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        response["nodeId"] = String(nodeId, HEX);
        response["nodeName"] = nodeName;
        response["uptime"] = millis() / 1000;
        response["powerSaving"] = powerSavingEnabled;
        response["debug"] = debugEnabled;
        response["debugLevel"] = debugLevel;
        
        int onlineCount = 0;
        for (int i = 0; i < TINYMESH_MAX_NODES; i++) {
            if (routingTable[i].id && routingTable[i].isOnline) onlineCount++;
        }
        response["connectedNodes"] = onlineCount;
    }
    else if (strcmp(cmd, "NAME") == 0) {
        if (doc.containsKey("value")) {
            setNodeName(doc["value"]);
            response["status"] = "ok";
        }
    }
    else if (strcmp(cmd, "PASSCODE") == 0) {
        if (doc.containsKey("value")) {
            setBLEPasscode(doc["value"]);
            response["status"] = "ok";
        }
    }
    else if (strcmp(cmd, "DEBUG") == 0) {
        if (doc.containsKey("enable")) {
            setDebugEnable(doc["enable"]);
        }
        if (doc.containsKey("level")) {
            setDebugLevel(doc["level"]);
        }
        response["status"] = "ok";
    }
    else if (strcmp(cmd, "POWER_SAVING") == 0) {
        if (doc.containsKey("value")) {
            powerSavingEnabled = doc["value"];
            preferences.putBool("powerSaving", powerSavingEnabled);
            response["status"] = "ok";
        }
    }
    else if (strcmp(cmd, "REBOOT") == 0) {
        response["status"] = "rebooting";
        sendResponse(response);
        delay(100);
        ESP.restart();
        return;
    }
    
    sendResponse(response);
}

void TinyMeshClass::debugLog(uint8_t level, const char* message) {
    if (debugEnabled && level <= debugLevel) {
        StaticJsonDocument<200> doc;
        doc["level"] = level;
        doc["message"] = message;
        String output;
        serializeJson(doc, output);
        Serial.println(output);
        
        if (deviceConnected && pCharacteristic) {
            pCharacteristic->setValue((uint8_t*)output.c_str(), output.length());
            pCharacteristic->notify();
        }
    }
}

void TinyMeshClass::sendResponse(JsonDocument& doc) {
    String output;
    serializeJson(doc, output);
    Serial.println(output);
    
    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue((uint8_t*)output.c_str(), output.length());
        pCharacteristic->notify();
    }
}

void TinyMeshClass::sendResponse(const char* jsonStr) {
    Serial.println(jsonStr);
    
    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue((uint8_t*)jsonStr, strlen(jsonStr));
        pCharacteristic->notify();
    }
}

uint32_t TinyMeshClass::generateNodeId() {
    uint32_t id = preferences.getUInt("nodeId", 0);
    if (id == 0 || id == NODENUM_RESERVED) {
        WiFi.mode(WIFI_MODE_STA);
        String mac = WiFi.macAddress();
        WiFi.mode(WIFI_OFF);
        
        mac.replace(":", "");
        id = (uint32_t)strtoul(mac.substring(6).c_str(), NULL, 16);
        if (id == 0 || id == NODENUM_RESERVED) {
            do {
                id = esp_random();
            } while (id == 0 || id == NODENUM_RESERVED);
        }
        preferences.putUInt("nodeId", id);
    }
    return id;
}

void TinyMeshClass::initializeChannels() {
    loadChannelsFromPreferences();
    
    if (!channels[0].enabled) {
        channels[0].id = 0;
        strcpy(channels[0].name, "Default");
        channels[0].enabled = true;
        channels[0].frequency = TINYMESH_DEFAULT_FREQUENCY;
        channels[0].bandwidth = 125;
        channels[0].spreadFactor = 7;
        channels[0].codingRate = 5;
        saveChannelToPreferences(0);
    }
}

void TinyMeshClass::loadChannelsFromPreferences() {
    for (int i = 0; i < TINYMESH_MAX_CHANNELS; i++) {
        String key = "channel" + String(i);
        if (preferences.isKey(key.c_str())) {
            size_t len = preferences.getBytesLength(key.c_str());
            if (len == sizeof(Channel)) {
                preferences.getBytes(key.c_str(), &channels[i], sizeof(Channel));
            }
        } else {
            channels[i].enabled = false;
        }
    }
}

void TinyMeshClass::saveChannelToPreferences(uint8_t channelNum) {
    if (channelNum < TINYMESH_MAX_CHANNELS) {
        String key = "channel" + String(channelNum);
        preferences.putBytes(key.c_str(), &channels[channelNum], sizeof(Channel));
    }
}

void TinyMeshClass::clearRoutingTable() {
    memset(routingTable, 0, sizeof(routingTable));
}

void TinyMeshClass::updateRoutingTable(uint32_t id, const char* name, int rssi, int8_t snr, uint8_t hops) {
    if (id == nodeId || id == NODENUM_RESERVED) return;
    
    int idx = -1;
    for (int i = 0; i < TINYMESH_MAX_NODES; i++) {
        if (routingTable[i].id == id) {
            idx = i;
            break;
        } else if (idx == -1 && routingTable[i].id == 0) {
            idx = i;
        }
    }
    
    if (idx != -1) {
        routingTable[idx].id = id;
        if (name) strncpy(routingTable[idx].name, name, 15);
        routingTable[idx].rssi = rssi;
        routingTable[idx].lastSNR = snr;
        routingTable[idx].lastHeard = millis();
        routingTable[idx].hops = hops;
        routingTable[idx].isOnline = true;
        routingTable[idx].distance = calculateDistance(rssi);
    }
}

void TinyMeshClass::updateNodeStatus() {
    unsigned long currentTime = millis();
    for (int i = 0; i < TINYMESH_MAX_NODES; i++) {
        if (routingTable[i].id && routingTable[i].isOnline) {
            if (currentTime - routingTable[i].lastHeard > TINYMESH_NODE_TIMEOUT) {
                routingTable[i].isOnline = false;
                debugLog(2, ("Node " + String(routingTable[i].id, HEX) + " went offline").c_str());
            }
        }
    }
}

float TinyMeshClass::calculateDistance(int rssi) {
    return pow(10, (-69 - (float)rssi) / (10 * 2.0));
}

void TinyMeshClass::managePower() {
    if (millis() - lastActivity > TINYMESH_INACTIVITY_TIMEOUT) {
        LoRa.sleep();
        if (TINYMESH_SLEEP_DURATION > 0) {
            esp_sleep_enable_timer_wakeup(TINYMESH_SLEEP_DURATION * 1000);
            esp_light_sleep_start();
        }
    }
}

bool TinyMeshClass::sendPacket(const TinyMeshPacket* packet) {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)packet, sizeof(TinyMeshPacket));
    bool success = LoRa.endPacket();
    if (success) {
        lastActivity = millis();
    }
    return success;
}

void TinyMeshClass::receivePackets() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        TinyMeshPacket packet;
        if (packetSize == sizeof(TinyMeshPacket)) {
            LoRa.readBytes((uint8_t*)&packet, sizeof(TinyMeshPacket));
            int rssi = LoRa.packetRssi();
            float snr = LoRa.packetSnr();
            processPacket(&packet, rssi, snr);
        }
    }
}

void TinyMeshClass::processPacket(TinyMeshPacket* packet, int rssi, float snr) {
    bool isForUs = (packet->to == nodeId || packet->to == NODENUM_RESERVED);
    updateRoutingTable(packet->from, NULL, rssi, snr, packet->hops);
    
    if (isForUs) {
        StaticJsonDocument<512> response;
        response["from"] = String(packet->from, HEX);
        response["type"] = packet->type;
        response["channel"] = packet->channel;
        response["msgId"] = packet->id;
        
        switch (packet->type) {
            case MSG_TYPE_DATA:
                response["message"] = String((char*)packet->payload);
                break;
            case MSG_TYPE_DISCOVERY:
                response["name"] = String((char*)packet->payload);
                break;
            case MSG_TYPE_POSITION:
                response["battery"] = packet->payload[0];
                break;
        }
        
        sendResponse(response);
    }
    
    if (!isForUs && packet->hops < TINYMESH_MAX_HOPS) {
        packet->hops++;
        delay(random(10, 100));
        sendPacket(packet);
    }
}

void TinyMeshClass::sendDiscoveryPacket() {
    TinyMeshPacket packet;
    packet.to = NODENUM_RESERVED;
    packet.from = nodeId;
    packet.type = MSG_TYPE_DISCOVERY;
    packet.channel = 0;
    packet.hops = 0;
    packet.flags = 0;
    packet.id = messageCounter++;
    memcpy(packet.payload, nodeName, strlen(nodeName) + 1);
    packet.payloadLen = strlen(nodeName) + 1;
    
    sendPacket(&packet);
}

void TinyMeshClass::sendPositionUpdate() {
    TinyMeshPacket packet;
    packet.to = NODENUM_RESERVED;
    packet.from = nodeId;
    packet.type = MSG_TYPE_POSITION;
    packet.channel = 0;
    packet.hops = 0;
    packet.flags = 0;
    packet.id = messageCounter++;
    packet.payload[0] = 100;
    packet.payloadLen = 1;
    
    sendPacket(&packet);
}

TinyMeshClass TinyMesh;