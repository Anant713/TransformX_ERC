#include "Robot.hpp"
#include <cmath>

Robot::Robot(IServoDriver* drv) : driver(drv)
{
}

void Robot::setPosition(float x, float y, float z)
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        footPos[i][0] = x;
        footPos[i][1] = y;
        footPos[i][2] = z;
    }
}

void Robot::setAngles(int leg, float t1, float t2, float t3)
{
    theta1[leg] = t1;
    theta2[leg] = t2;
    theta3[leg] = t3;
}

bool Robot::computeIK()
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        float x = footPos[i][0] - xl[i];
        float y = footPos[i][1] - yl[i];
        float z = footPos[i][2] - zl[i];

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
        return false;

    t3 = atan2(sqrt(1 - D * D), D);
    t2 = atan2(-z, r - RobotConfig::a) -
         atan2(RobotConfig::c * sin(t3), RobotConfig::b + RobotConfig::c * cos(t3));

    t1 = atan2(x, y);

    t1 *= RAD_TO_DEG;
    t2 *= RAD_TO_DEG;
    t3 *= RAD_TO_DEG;

    return true;
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

void Robot::update()
{
    if (mode == MODE_POSITION)
    {
        if (!computeIK())
            return;
    }

    applyServoMapping();
    driver->writeAll(servo1, servo2, servo3);
}