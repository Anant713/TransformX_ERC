#include <Arduino.h>
#include <stdio.h>
#include "RobotConfig.hpp"
#include "Communication.hpp"
#include "RobotState.hpp"


CommandType receiveCommand()
{
    if (!Serial.available())
        return CMD_NONE;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "home")
    {
        theta1_out[0] = 90.0f;
        theta1_out[1] = 90.0f;
        theta1_out[2] = 90.0f;
        theta1_out[3] = 90.0f;

        theta2_out[0] = 45.0f;
        theta2_out[1] = 45.0f;
        theta2_out[2] = 45.0f;
        theta2_out[3] = 45.0f;

        theta3_out[0] = 45.0f;
        theta3_out[1] = 45.0f;
        theta3_out[2] = 45.0f;
        theta3_out[3] = 45.0f;

        Serial.println("Going Home.");
        return CMD_HOME;
    }

    if (cmd.startsWith("setpos"))
    {
        float x, y, z;
        int parsed = sscanf(cmd.c_str(), "setpos %f %f %f", &x, &y, &z);

        if (parsed != 3)
        {
            Serial.println("Invalid setpos format. Use: setpos x y z");
            return CMD_NONE;
        }

        for (int i = 0; i < NUM_LEGS; i++)
        {
            legEndPos[i][0] = x;
            legEndPos[i][1] = y;
            legEndPos[i][2] = z;
        }

        Serial.println("New Position Set.");
        return CMD_SET_POS;
    }

    if (cmd.startsWith("setang"))
    {
        float theta1, theta2, theta3;
        int parsed = sscanf(cmd.c_str(), "setang %f %f %f", &theta1, &theta2, &theta3);

        if (parsed != 3)
        {
            Serial.println("Invalid setang format. Use: setang theta1 theta2 theta3");
            return CMD_NONE;
        }

        theta1_out[0] = theta1;
        theta1_out[1] = 180.0f - theta1;
        theta1_out[2] = theta1;
        theta1_out[3] = 180.0f - theta1;

        theta2_out[0] = theta2;
        theta2_out[1] = -theta2;
        theta2_out[2] = theta2;
        theta2_out[3] = -theta2;

        theta3_out[0] = theta3;
        theta3_out[1] = -theta3;
        theta3_out[2] = theta3;
        theta3_out[3] = -theta3;

        Serial.println("New Angles Set.");
        return CMD_SET_ANG;
    }

    Serial.println("Unknown command.");
    return CMD_NONE;
}