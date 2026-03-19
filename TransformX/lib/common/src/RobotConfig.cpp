#include "RobotConfig.hpp"

float a = 60.0f;
float b = 95.0f;
float c = 180.0f;

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

void initRobotConfig()
{
    // Keep for future initialization if needed.
}