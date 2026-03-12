#include "RobotConfig.hpp"
#include "Kinematics.hpp"
#include "ServoController.hpp"
#include "Communication.hpp"



void setup()
{
    Serial.begin(115200);
    //initRobotConfig();
    initServos();
    Serial.println("Quadruped Ready.");
    Serial.println("Commands:");
    Serial.println("home");
    Serial.println("set x y z  (sets all legs)");
}

void loop()
{
    if (receiveCommand())
    {
        if (inv_ken_global(legEndPos, inv))
        {
            for ( int i=0; i<NUM_LEGS ; i++){
                Serial.println(theta3_out[i]);
                Serial.println(theta1_out[i]);
                Serial.println(theta2_out[i]);
            }
            writeServos(theta1_out, theta2_out, theta3_out);
        }
        else
        {
            Serial.println("Unreachable position, IK not done!");
        }
    }
}