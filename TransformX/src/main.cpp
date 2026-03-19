#include <Arduino.h>
#include "RobotConfig.hpp"
#include "Kinematics.hpp"
#include "ServoController.hpp"
#include "Communication.hpp"
#include "HardwareConfig.hpp"

static void printJointAngles()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        Serial.print("Leg ");
        Serial.print(i);
        Serial.print(" -> ");

        Serial.print(theta1_out[i]);
        Serial.print(", ");
        Serial.print(theta2_out[i]);
        Serial.print(", ");
        Serial.println(theta3_out[i]);
    }
}

static void applyServoMapping()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        theta1_out[i] = servoOffsets[i][0] + theta1_out[i] * servoAngSigns[i][0];
        theta2_out[i] = servoOffsets[i][1] + theta2_out[i] * servoAngSigns[i][1];
        theta3_out[i] = servoOffsets[i][2] + theta3_out[i] * servoAngSigns[i][2];
    }
}

void setup()
{
    Serial.begin(115200);
    initServos();

    Serial.println("Quadruped Ready.");
    Serial.println("Commands:");
    Serial.println("home");
    Serial.println("setpos x y z");
    Serial.println("setang t1 t2 t3");
}

void loop()
{
    CommandType cmdType = receiveCommand();

    if (cmdType == CMD_NONE)
        return;

    bool success = true;

    if (cmdType == CMD_SET_POS)
    {
        success = inv_ken_global(legEndPos);
        if (!success)
        {
            Serial.println("Unreachable position, IK not done!");
            return;
        }
    }

    printJointAngles();
    applyServoMapping();
    writeServosDriver(theta1_out, theta2_out, theta3_out);
}