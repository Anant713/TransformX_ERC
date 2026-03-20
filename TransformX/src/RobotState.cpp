#include "RobotConfig.hpp"
#include "RobotState.hpp"


float xl[NUM_LEGS] = {0.0f, 0.0f, 0.0f, 0.0f};
float yl[NUM_LEGS] = {0.0f, 0.0f, 0.0f, 0.0f};
float zl[NUM_LEGS] = {0.0f, 0.0f, 0.0f, 0.0f};

int sideSign[NUM_LEGS] = {1, -1, 1, -1};

float servoAngSigns[NUM_LEGS][NUM_JOINTS] = {
    {-1.0f, -1.0f,  1.0f},
    { 0.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f}
};

float servoOffsets[NUM_LEGS][NUM_JOINTS] = {
    {90.0f, 90.0f, 90.0f},
    { 0.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f}
};

float legEndPos[NUM_LEGS][NUM_JOINTS] = {0.0f};

float theta1_out[NUM_LEGS]= {0.0f};
float theta2_out[NUM_LEGS] = {0.0f};
float theta3_out[NUM_LEGS] = {0.0f};

void initRobotConfig()
{
    // Keep for future initialization if needed.
}