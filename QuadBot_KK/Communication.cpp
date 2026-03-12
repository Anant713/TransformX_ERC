#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "RobotConfig.hpp"
#include "Communication.hpp"
#include "Kinematics.hpp"
#include "ServoController.hpp"

// ─────────────────────────────────────────────
//  WiFi credentials — ESP32 runs as access point
// ─────────────────────────────────────────────
const char* AP_SSID     = "RobotDog-Debug";
const char* AP_PASSWORD = "transformx";

WebSocketsServer webSocket(81);

// ─────────────────────────────────────────────
//  Helper: send a message to ALL outputs
// ─────────────────────────────────────────────
static void broadcast(const String& msg) {
    Serial.println(msg);
    webSocket.broadcastTXT(msg);
}

// ─────────────────────────────────────────────
//  Execute a command and move servos if needed
// ─────────────────────────────────────────────
bool parseCommand(const String& rawCmd) {
    String cmd = rawCmd;
    cmd.trim();
    if (cmd.length() == 0) return false;

    // ── home ──────────────────────────────────
    if (cmd == "home") {
        legEndPos[activeLeg][0] = homePos[0];
        legEndPos[activeLeg][1] = homePos[1];
        legEndPos[activeLeg][2] = homePos[2];
        inv = true;

        broadcast("CMD: home → leg " + String(activeLeg) +
                  " pos(" + homePos[0] + "," + homePos[1] + "," + homePos[2] + ")");
        return true;
    }

    // ── setleg <index> ────────────────────────
    if (cmd.startsWith("setleg")) {
        int leg;
        if (sscanf(cmd.c_str(), "setleg %d", &leg) == 1
            && leg >= 0 && leg < NUM_LEGS) {
            activeLeg = leg;
            broadcast("Active leg set to " + String(activeLeg));
        } else {
            broadcast("ERR: setleg <0-3>");
        }
        return true;
    }

    // ── setpos <x> <y> <z> ───────────────────
    if (cmd.startsWith("setpos")) {
        float x, y, z;
        if (sscanf(cmd.c_str(), "setpos %f %f %f", &x, &y, &z) == 3) {
            legEndPos[activeLeg][0] = x;
            legEndPos[activeLeg][1] = y;
            legEndPos[activeLeg][2] = z;
            inv = true;
            broadcast("CMD: setpos leg " + String(activeLeg) +
                      " → (" + x + "," + y + "," + z + ")");
        } else {
            broadcast("ERR: setpos <x> <y> <z>");
        }
        return true;
    }

    // ── setang <t1> <t2> <t3> ────────────────
    if (cmd.startsWith("setang")) {
        float t1, t2, t3;
        if (sscanf(cmd.c_str(), "setang %f %f %f", &t1, &t2, &t3) == 3) {
            theta1_out[activeLeg] = t1;
            theta2_out[activeLeg] = t2;
            theta3_out[activeLeg] = t3;
            inv = false;
            broadcast("CMD: setang leg " + String(activeLeg) +
                      " → t1=" + t1 + " t2=" + t2 + " t3=" + t3);
        } else {
            broadcast("ERR: setang <t1> <t2> <t3>");
        }
        return true;
    }

    // ── sethome <x> <y> <z> ──────────────────
    if (cmd.startsWith("sethome")) {
        float x, y, z;
        if (sscanf(cmd.c_str(), "sethome %f %f %f", &x, &y, &z) == 3) {
            homePos[0] = x;
            homePos[1] = y;
            homePos[2] = z;
            broadcast("Home updated → (" + String(x) + "," + String(y) + "," + String(z) + ")");
        } else {
            broadcast("ERR: sethome <x> <y> <z>");
        }
        return true;
    }

    // ── testik <x> <y> <z> ───────────────────
    // Computes and prints IK angles WITHOUT moving servos
    if (cmd.startsWith("testik")) {
        float x, y, z;
        if (sscanf(cmd.c_str(), "testik %f %f %f", &x, &y, &z) == 3) {
            testIK(x, y, z, activeLeg);
        } else {
            broadcast("ERR: testik <x> <y> <z>");
        }
        return true;
    }

    // ── getpos ────────────────────────────────
    if (cmd == "getpos") {
        broadcast("Leg " + String(activeLeg) +
                  " pos=(" + legEndPos[activeLeg][0] +
                  "," + legEndPos[activeLeg][1] +
                  "," + legEndPos[activeLeg][2] + ")" +
                  " angles t1=" + theta1_out[activeLeg] +
                  " t2=" + theta2_out[activeLeg] +
                  " t3=" + theta3_out[activeLeg]);
        return true;
    }

    // ── getconfig ─────────────────────────────
    if (cmd == "getconfig") {
        broadcast("Links: a=" + String(a) + " b=" + String(b) + " c=" + String(c));
        broadcast("Home: (" + String(homePos[0]) + "," +
                  String(homePos[1]) + "," + String(homePos[2]) + ")");
        for (int i = 0; i < NUM_LEGS; i++) {
            broadcast("Leg" + String(i) + " hip=(" +
                      xl[i] + "," + yl[i] + "," + zl[i] +
                      ") sign=" + sideSign[i]);
        }
        return true;
    }

    // ── status ────────────────────────────────
    if (cmd == "status") {
        broadcast("=== STATUS ===");
        broadcast("Active leg: " + String(activeLeg));
        broadcast("Home: (" + String(homePos[0]) + "," +
                  String(homePos[1]) + "," + String(homePos[2]) + ")");
        broadcast("Current pos: (" + String(legEndPos[activeLeg][0]) + "," +
                  String(legEndPos[activeLeg][1]) + "," +
                  String(legEndPos[activeLeg][2]) + ")");
        broadcast("Current angles: t1=" + String(theta1_out[activeLeg]) +
                  " t2=" + String(theta2_out[activeLeg]) +
                  " t3=" + String(theta3_out[activeLeg]));
        broadcast("Mode: " + String(inv ? "IK (inverse)" : "Direct angles"));
        return true;
    }

    // ── help ──────────────────────────────────
    if (cmd == "help") {
        broadcast("Commands:");
        broadcast("  setleg <0-3>          — select active leg");
        broadcast("  setpos <x> <y> <z>   — move leg via IK");
        broadcast("  setang <t1> <t2> <t3>— move leg via direct angles");
        broadcast("  sethome <x> <y> <z>  — redefine home position");
        broadcast("  home                 — go to home position");
        broadcast("  testik <x> <y> <z>   — compute IK, don't move");
        broadcast("  getpos               — print current position+angles");
        broadcast("  getconfig            — print robot config");
        broadcast("  status               — print full status");
        return true;
    }

    broadcast("Unknown command: " + cmd + " (type 'help')");
    return false;
}

// ─────────────────────────────────────────────
//  WebSocket event handler
// ─────────────────────────────────────────────
void onWebSocketEvent(uint8_t clientNum, WStype_t type,
                      uint8_t* payload, size_t length)
{
    switch (type) {
        case WStype_CONNECTED:
            broadcast("WebSocket client " + String(clientNum) + " connected.");
            broadcast("Type 'help' for commands.");
            break;

        case WStype_DISCONNECTED:
            Serial.println("WebSocket client " + String(clientNum) + " disconnected.");
            break;

        case WStype_TEXT: {
            String cmd = String((char*)payload);
            bool valid = parseCommand(cmd);
            if (valid) {
                // Execute motion after parsing
                if (inv_ken_global(legEndPos, inv, activeLeg)) {
                    writeServos(theta1_out, theta2_out, theta3_out);
                    broadcast("OK: servos updated.");
                } else {
                    broadcast("ERR: IK failed, servos NOT moved.");
                }
            }
            break;
        }

        default:
            break;
    }
}

// ─────────────────────────────────────────────
//  Init
// ─────────────────────────────────────────────
void initCommunication() {
    // Serial always available as fallback
    Serial.println("Serial ready.");

    // Start WiFi access point
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("WiFi AP started. Connect to: ");
    Serial.println(AP_SSID);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    // Start WebSocket server on port 81
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    Serial.println("WebSocket server started on port 81.");
    Serial.println("Type 'help' for commands.");
}

// ─────────────────────────────────────────────
//  Main loop handler — call every loop()
// ─────────────────────────────────────────────
void handleCommunication() {
    // Handle WebSocket
    webSocket.loop();

    // Handle Serial
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        bool valid = parseCommand(cmd);
        if (valid) {
            if (inv_ken_global(legEndPos, inv, activeLeg)) {
                writeServos(theta1_out, theta2_out, theta3_out);
                broadcast("OK: servos updated.");
            } else {
                broadcast("ERR: IK failed, servos NOT moved.");
            }
        }
    }
}
