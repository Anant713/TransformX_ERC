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
        sscanf(input.c_str(), "setpos %f %f %f", &cmd.x, &cmd.y, &cmd.z);
        cmd.type = CMD_SET_POS;
    }
    else if (input.startsWith("setang"))
    {
        sscanf(input.c_str(), "setang %d %f %f %f",
               &cmd.leg, &cmd.t1, &cmd.t2, &cmd.t3);
        cmd.type = CMD_SET_ANG;
    }
    else if (input == "walk")
    {
        cmd.type = CMD_WALK;
    }

    return cmd;
}