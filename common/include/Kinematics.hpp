#pragma once

#include "RobotConfig.hpp"


bool inv_kin_global(const float legEndPos[NUM_LEGS][NUM_JOINTS],
        const float xl[NUM_LEGS],
        const float yl[NUM_LEGS],
        const float zl[NUM_LEGS],
        const int sideSign[NUM_LEGS],
        float theta1_out[NUM_LEGS],
        float theta2_out[NUM_LEGS],
        float theta3_out[NUM_LEGS]);
bool inverseKinematics(
        float x, float y, float z,
        float a, float b, float c,
        float& theta1,
        float& theta2,
        float& theta3);