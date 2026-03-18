#include <Arduino.h>
#include "RobotConfig.hpp"
#include "Kinematics.hpp"

float theta1_out[4];
float theta2_out[4];
float theta3_out[4];

bool inverseKinematics(float &x, float &y, float &z,
                       float &theta1,
                       float &theta2,
                       float &theta3)
{
    float r = sqrt(x*x + y*y);

    float D = ((r - a)*(r - a) + z*z - b*b - c*c) / (2*b*c);
    
    if (D > 1.0 || D < -1.0){
        Serial.println("D value out of bound ( that is theta 2)");
        Serial.println(D);
        return false;
    }
    theta3 = atan2(sqrt(1 - D*D), D);

    theta2 = atan2(-z, r - a)
             - atan2(c*sin(theta3),
                     b + c*cos(theta3));

    theta1 = atan2(x, y);

    // convert to degrees
    theta1 *= 180.0 / PI;
    theta2 *= 180.0 / PI;
    theta3 *= 180.0 / PI;
    return true;
}

bool inv_ken_global(float legEndPos[4][3], bool &inv)
{   
    Serial.println("Running global kinematics");
    for (int i = 0; i < NUM_LEGS; i++)
    {
        if (inv){
            Serial.println("Running Inverse Kinematics");
            float x_local = legEndPos[i][0] - xl[i];
            float y_local = legEndPos[i][1] - yl[i];
            float z_local = legEndPos[i][2] - zl[i];

            y_local *= sideSign[i];
        
            bool reachable = inverseKinematics(
                                x_local,
                                y_local,
                                z_local,
                                theta1_out[i],
                                theta2_out[i],
                                theta3_out[i]);
            if (!reachable)
                return false;
        }

        theta1_out[i] *= sideSign[i];

    }

    return true;
}