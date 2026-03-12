#include <Arduino.h>
#include "RobotConfig.hpp"
#include "Communication.hpp"
#include "Kinematics.hpp"
#include "ServoController.hpp"
bool receiveCommand()
{
    if (!Serial.available())
        return false;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "home")
    {
        theta1_out[0] = 45;
        theta1_out[1] = 0;
        theta1_out[2] = 0;
        theta1_out[3] = 0;

        theta2_out[0] = 45;
        theta2_out[1] = 0;
        theta2_out[2] = 0;
        theta2_out[3] = 0;

        theta3_out[0] = 90;
        theta3_out[1] = 0;
        theta3_out[2] = 0;
        theta3_out[3] = 0;      

        inv = false ;
        //writeServos(theta1_out, theta2_out, theta3_out);
        // for (int i = 0; i < NUM_LEGS; i++)
        // {
        //     legEndPos[i][0] = 45;
        //     legEndPos[i][1] = 135;
        //     legEndPos[i][2] = 90;
        // }

        Serial.println("Going Home.");
        return true;
    }

    if (cmd.startsWith("setpos"))
    {
        //Serial.println("You just set a custom position");
        float x, y, z;
        sscanf(cmd.c_str(), "setpos %f %f %f", &x, &y, &z);

        for (int i = 0; i < NUM_LEGS; i++)
        {
            legEndPos[i][0] = x;
            legEndPos[i][1] = y;
            legEndPos[i][2] = z;
        }
        inv = true;
        Serial.println("New Position Set.");
        return true;
    }

    if (cmd.startsWith("setang"))
    {
        //Serial.println("You just set a custom position");
        //float theta3, theta1, theta2;
        sscanf(cmd.c_str(), "setang %f %f %f", &theta3_out[0], &theta1_out[0], &theta2_out[0]);

        // for (int i = 0; i < NUM_LEGS; i++)
        // {
        //     legEndPos[i][0] = x;
        //     legEndPos[i][1] = y;
        //     legEndPos[i][2] = z;
        // }
        inv  = false ;
        Serial.println("New Angle set .");
        return true;
    }

    return false;
}