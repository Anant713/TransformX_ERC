#pragma once

#include "IServoDriver.hpp"
#include "RobotConfig.hpp"
constexpr int RAD_TO_DEG = 180.0f / 3.14159265f;


/**
 * @brief Main robot class (application layer)
 *
 * Owns:
 * - State (joint angles, foot positions)
 * - Config (geometry, offsets)
 * - Control logic (IK, gait, etc.)
 */
class Robot
{
private:
    // --- State ---
    float footPos[NUM_LEGS][NUM_JOINTS];
    float theta1[NUM_LEGS];
    float theta2[NUM_LEGS];
    float theta3[NUM_LEGS];

    // Servo-mapped angles
    float servo1[NUM_LEGS];
    float servo2[NUM_LEGS];
    float servo3[NUM_LEGS];

    // --- Geometry ---
    float xl[NUM_LEGS];
    float yl[NUM_LEGS];
    float zl[NUM_LEGS];

    // Driver abstraction
    IServoDriver* driver;

    // --- Private methods ---
    bool computeIK();
    bool inverseKinematics(float x, float y, float z,
                           float& t1, float& t2, float& t3);

    void applyServoMapping();

public:
    enum Mode
    {
        MODE_IDLE,
        MODE_POSITION,
        MODE_WALK
    };

    Mode mode;

    Robot(IServoDriver* drv);

    // --- Public API ---
    void setPosition(float x, float y, float z);
    void setAngles(int leg, float t1, float t2, float t3);

    void update();
};