#include "Robot.hpp"
#include <Arduino.h>  
#include <cmath>



// --- Private methods, internal working of bot, not exposed to user ---
Robot::Robot(IServoDriver* servodrv, IIMU* imudrv){
    servoDriver = servodrv;
    imu = imudrv;
}

void Robot::LinearTransition(int leg, int steps, int delayMs, float changex, float changey, float changez)
{
    float stepX = changex / steps;
    float stepY = changey / steps;
    float stepZ = changez / steps;

    for (int i = 0; i < steps; i++)
    {
        footPos[leg][0] += stepX;
        footPos[leg][1] += stepY;
        footPos[leg][2] += stepZ;

        if (!computeIK()){
            Serial.println("IK failed during smooth transition!");
            mode = MODE_IDLE;
            return;
        }

        update();
        delay(delayMs);
    }
}

bool Robot::computeIK()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        float x = footPos[i][0] - RobotConfig::xl[i];
        float y = footPos[i][1] - RobotConfig::yl[i];
        float z = footPos[i][2] - RobotConfig::zl[i];

        if (!inverseKinematics(x, y, z,
                               theta1[i], theta2[i], theta3[i]))
        {
            return false;
        }
    }
    return true;
}

bool Robot::inverseKinematics(
    float x, float y, float z,
    float& t1, float& t2, float& t3)
{
    float r = sqrt(x * x + y * y);

    float D = ((r - RobotConfig::a) * (r - RobotConfig::a) + z * z - RobotConfig::b * RobotConfig::b - RobotConfig::c * RobotConfig::c) / (2 * RobotConfig::b * RobotConfig::c);

    if (D > 1.0f || D < -1.0f)
    {
        Serial.println("D value out of range for IK, leg might be out of reach");
        return false;
    }


    t3 = atan2(sqrt(1 - D * D), D);
    t2 = atan2(-z, r - RobotConfig::a) -
         atan2(RobotConfig::c * sin(t3), RobotConfig::b + RobotConfig::c * cos(t3));

    t1 = atan2(x, y);

    t1 *= RAD_TO_DEG;
    t2 *= RAD_TO_DEG;
    t3 *= RAD_TO_DEG;

    return true;
}

bool Robot::forwardKinematics(float t1, float t2, float t3,
                              float& x, float& y, float& z)
{
    // Convert to radians
    float th1 = t1 * DEG_TO_RAD;
    float th2 = t2 * DEG_TO_RAD;
    float th3 = t3 * DEG_TO_RAD;

    // Z
    z = - (RobotConfig::b * sin(th2) + RobotConfig::c * sin(th2 + th3));

    // Planar projection
    float R = RobotConfig::a + RobotConfig::b * cos(th2) + RobotConfig::c * cos(th2 + th3);

    // X, Y
    x = sin(th1) * R;
    y = cos(th1) * R;

    return true;
}

// --- Public API, user calls these to control the bot ---

void Robot::idle()
{
    mode = MODE_IDLE;
}

void Robot::home(float x, float y, float z)
{
    mode = MODE_POSITION;
    for (int i = 0; i < NUM_LEGS; i++)
    {
        footPos[i][0] = x;
        footPos[i][1] = y;
        footPos[i][2] = z;
    }
    if (!computeIK()){
        Serial.println("IK failed for given position!");
        mode = MODE_IDLE;
        return;
    }  
}

void Robot::setPosition(int leg,float x, float y, float z)
{
    mode = MODE_POSITION;
    footPos[leg][0] = x;
    footPos[leg][1] = y;
    footPos[leg][2] = z;
    if (!computeIK()){
        Serial.println("IK failed for given position!");
        mode = MODE_IDLE;
        return;
    }
}

void Robot::setAngles(int leg, float t1, float t2, float t3)
{
    mode = MODE_ANGLE;
    theta1[leg] = t1;
    theta2[leg] = t2;
    theta3[leg] = t3;
}

void Robot::walk()
{
    mode = MODE_WALK;
    // Move FL,BR leg 5cm up 5 cm forward
    printFootState();
    delay(1000);
    LinearTransition(0,50,10,50,0,50);
    LinearTransition(3,50,10,50,0,50);
    printFootState();
    //delay(1000);
    // Move FL,BR left leg down, total 10cm forward
    LinearTransition(0,50,10,50,0,-50);
    LinearTransition(3,50,10,50,0,-50);
    printFootState();
    //delay(1000);
    // Move body forward by 5 cm => All legs backward by 5cm
    for (int i=0; i<NUM_LEGS; i++)
    {
        LinearTransition(i,50,10,-50,0,0);
    }
    printFootState();
    //delay(1000);
    // Move FR,BL leg 5cm up 5 cm forward
    LinearTransition(1,50,10,50,0,50);
    LinearTransition(2,50,10,50,0,50);
    printFootState();
    //delay(1000);
    // Move FR,BL left leg down, total 10cm forward
    LinearTransition(1,50,10,50,0,-50);
    LinearTransition(2,50,10,50,0,-50);
    printFootState();
    //delay(1000);
}

void Robot::update()
{

    if (mode == MODE_IDLE) return;
    if (mode == MODE_POSITION)
    {
        Serial.println("State : POSITION");  
        printServoAngles();
    }
    //if (mode == MODE_IDLE) Serial.println("State : IDLE");
    if (mode == MODE_WALK)
    {
        Serial.println("State : WALK");
    }
    if (mode == MODE_ANGLE)
    {
        Serial.println("State : ANGLE");
        printServoAngles();

    }
    applyServoMapping();
    //printServoAngles();
    servoDriver->writeAll(servo1, servo2, servo3);
}


void Robot::applyServoMapping()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        servo1[i] = RobotConfig::servoOffsets[i][0] + theta1[i] * RobotConfig::servoSigns[i][0];
        servo2[i] = RobotConfig::servoOffsets[i][1] + theta2[i] * RobotConfig::servoSigns[i][1];
        servo3[i] = RobotConfig::servoOffsets[i][2] + theta3[i] * RobotConfig::servoSigns[i][2];
    }
}


// --- Public methodsFor debugging, print different states
void Robot::printFootState()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        Serial.print("Leg ");
        Serial.print(i);
        Serial.print(" Foot Position -> ");

        Serial.print(footPos[i][0]);
        Serial.print(" ,");
        Serial.print(footPos[i][1]);
        Serial.print(" ,");
        Serial.println(footPos[i][2]);
    }
}

void Robot::printAngleState()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        Serial.print("Leg ");
        Serial.print(i);
        Serial.print(" Joint Angles -> ");

        Serial.print(theta1[i]);
        Serial.print(" ,");
        Serial.print(theta2[i]);
        Serial.print(" ,");
        Serial.println(theta3[i]);
    }
}

void Robot::printServoAngles()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        Serial.print("Leg ");
        Serial.print(i);
        Serial.print(" Servo Angles -> ");

        Serial.print(servo1[i]);
        Serial.print(" ,");
        Serial.print(servo2[i]);
        Serial.print(" ,");
        Serial.println(servo3[i]);
    }
}