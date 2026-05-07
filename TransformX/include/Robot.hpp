#pragma once

#include "IServoDriver.hpp"
#include "RobotConfig.hpp"
constexpr float RAD_TO_DEG = 180.0f / 3.14159265f;
constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;


/**
 * @brief Main robot class (application layer)
 *
 * Owns:
 * - State (joint angles, foot positions)
 * - Config (geometry, offsets)
 * - Control logic (IK, gait, etc.)
 * The robot has a global frame attached to the centre of its main body, with x forward, y left, z up.
 * Each leg has its own local frame, with x forward, y left, z up on the left side and x forward, y right, z up on the right side.
 * The angles of each joint are given as the acute angle(+ or -) between the two links it connects, with positive being downward and negative being upward.
 */
class Robot
{
private: // Contains private state and methods(primarily computation, which the user doesn't need to access directly), public API is below
    // --- State ---
    float footPos[NUM_LEGS][NUM_JOINTS];// Foot positon in robot frame (x,y,z)
    float theta1[NUM_LEGS];  // Shoulder (hip) angles
    float theta2[NUM_LEGS];  // Elbow (knee) angles
    float theta3[NUM_LEGS];  // Wrist (ankle) angles 

    // --- Servo-mapped angles ---
    // Angles sent to servo after mapping, this mapping is fixed in hardware
    float servo1[NUM_LEGS]; 
    float servo2[NUM_LEGS];
    float servo3[NUM_LEGS];

    // Driver abstraction
    IServoDriver* driver;

    // --- Private methods ---
    bool computeIK(); // Compute IK for all legs, returns false if any leg is out of reach, calls inverseKinematics for each leg after adjusting for shoulder position
    bool inverseKinematics(float x, float y, float z,          // Compute IK for a single leg, returns false if out of reach
                           float& t1, float& t2, float& t3);
    bool forwardKinematics(float t1, float t2, float t3,       // Compute FK for a single leg, returns foot position in robot frame
                           float& x, float& y, float& z);
    
    void applyServoMapping(); // Apply servo mapping to all legs, converts theta1/2/3 to servo1/2/3 using RobotConfig

public:
    enum Mode // State machine bot
    {
        MODE_IDLE, 
        MODE_POSITION,
        MODE_ANGLE,
        MODE_WALK
    };

    Mode mode;

    Robot(IServoDriver* drv);

    // --- Public API ---
    void idle(); // User wants the bot to do nothing, it will hold its position and not update servo angles
    void home(float x, float y, float z); // User wants the bot to go home, It uses IK i.e. home position is set using foor position and not joint angles
    void setPosition(int leg, float x, float y, float z); // User wants any particular foot to go to a particular position
    void setAngles(int leg, float t1, float t2, float t3); // User wants any particular joint angles to be set
    void printFootState(); // When setting joint angles, expected leg position is also printed
    void printAngleState(); // When setting foot position, expected joint angles are also printed
    void printServoAngles(); // For debugging, print servo-mapped angles
    void walk(); // Contains a fixed gait of walking right now
    void update();
};