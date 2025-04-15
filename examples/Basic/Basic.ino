#include <TinyMesh.h>

void setup() {
    // Optional: Set BLE passcode and node name
    TinyMesh.setBLEPasscode(123456);
    TinyMesh.setNodeName("MyNode");
    
    if (!TinyMesh.begin()) {
        Serial.println(F("TinyMesh initialization failed"));
        while (1);
    }
}

void loop() {
    TinyMesh.update();
}