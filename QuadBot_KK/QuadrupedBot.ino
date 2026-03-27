#include "RobotConfig.hpp"
#include "Kinematics.hpp"
#include "ServoController.hpp"
#include "Communication.hpp"

// NOTE: Uses dual-core to keep WiFi stable.
// Core 0 → WiFi + WebSocket (handled by ESP32 internally)
// Core 1 → Robot loop (default Arduino loop core)

void setup() {
    Serial.begin(115200);
    initRobotConfig();   // sets all legs to homePos
    initServos();
    initCommunication(); // starts WiFi AP + WebSocket server
}

void loop() {
    handleCommunication(); // checks both Serial and WebSocket
}
