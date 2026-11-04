#include "Communication.hpp"
#include <Arduino.h>

Command receiveCommand()
{
    Command cmd;
    cmd.type = CMD_NONE;

    if (!Serial.available())
        return cmd;

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "home")
    {
        cmd.type = CMD_HOME;
    }
    else if (input.startsWith("setpos"))
    {
        int parsed = sscanf(input.c_str(), "setpos %d %f %f %f", 
                    &cmd.leg, &cmd.x, &cmd.y, &cmd.z);
        if (parsed != 4) {
            Serial.println("Invalid setpos command format. Expected: setpos <leg> <x> <y> <z>");
            return cmd;
        }
        cmd.type = CMD_SET_POS;
    }
    else if (input.startsWith("setang"))
    {
        int parsed = sscanf(input.c_str(), "setang %d %f %f %f",
               &cmd.leg, &cmd.t1, &cmd.t2, &cmd.t3);
        if (parsed != 4) {
            Serial.println("Invalid setang command format. Expected: setang <leg> <t1> <t2> <t3>");
            return cmd;
        }
        cmd.type = CMD_SET_ANG;
        Serial.println("Received joint angles");
    }
    else if (input == "walk")
    {
        cmd.type = CMD_WALK;
        Serial.println("Received walk command");
    }

    return cmd;
}