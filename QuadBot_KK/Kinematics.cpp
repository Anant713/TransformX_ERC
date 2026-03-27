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


    if (D > 1.0 || D < -1.0) {
        Serial.print("IK error: D out of bounds = ");
        Serial.println(D);
        return false;
    }

    theta2 = atan2(sqrt(1 - D*D), D);

    theta1 = atan2(-z, r - a)
             - atan2(c * sin(theta2),
                     b + c * cos(theta2));

    theta3 = atan2(x, y);

    // Convert to degrees
    theta1 *= 180.0 / PI;
    theta2 *= 180.0 / PI;
    theta3 *= 180.0 / PI;

    return true;
}

// Run IK for all legs (inv=true) or just apply existing angles (inv=false)
// legIndex = -1 means all legs, otherwise only the specified leg
bool inv_ken_global(float legEndPos[4][3], bool &inv, int legIndex)
{
    int start = (legIndex < 0) ? 0 : legIndex;
    int end   = (legIndex < 0) ? NUM_LEGS : legIndex + 1;

    for (int i = start; i < end; i++)
    {
        if (inv) {
            float x_local = legEndPos[i][0] - xl[i];
            float y_local = legEndPos[i][1] - yl[i];
            float z_local = legEndPos[i][2] - zl[i];

            y_local *= sideSign[i];

            bool reachable = inverseKinematics(
                x_local, y_local, z_local,
                theta1_out[i],
                theta2_out[i],
                theta3_out[i]);

            if (!reachable) {
                Serial.print("Leg ");
                Serial.print(i);
                Serial.println(" unreachable.");
                return false;
            }
        }

        theta3_out[i] *= sideSign[i];
        theta2_out[i] = 180 - theta2_out[i];
    }

    return true;
}

// Run IK and print computed angles WITHOUT moving servos — for verification
void testIK(float x, float y, float z, int legIndex)
{
    float t1, t2, t3;

    float x_local = x - xl[legIndex];
    float y_local = (y - yl[legIndex]) * sideSign[legIndex];
    float z_local = z - zl[legIndex];

    Serial.println("--- IK Test (no servo move) ---");
    Serial.print("Input  x="); Serial.print(x);
    Serial.print(" y=");       Serial.print(y);
    Serial.print(" z=");       Serial.println(z);

    bool ok = inverseKinematics(x_local, y_local, z_local, t1, t2, t3);

    if (ok) {
        t3 *= sideSign[legIndex];
        t2  = 180 - t2;
        Serial.print("theta1="); Serial.print(t1);
        Serial.print(" theta2="); Serial.print(t2);
        Serial.print(" theta3="); Serial.println(t3);
    } else {
        Serial.println("Position unreachable.");
    }
}
