#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <Arduino.h>

// Call once in setup()
void initCommunication();

// Call every loop() — checks both Serial and WebSocket for commands
void handleCommunication();

// Parse and execute a command string from any source
// Returns true if a valid command was received
bool parseCommand(const String& cmd);

#endif
