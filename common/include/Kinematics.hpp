#pragma once

#include "RobotConfig.hpp"

namespace my_bot_ik
{       
bool inv_kin_global(const float legEndPos[NUM_LEGS][NUM_JOINTS],
        const float xl[NUM_LEGS],
        const float yl[NUM_LEGS],
        const float zl[NUM_LEGS],
        const int sideSign[NUM_LEGS],
        float theta1_out[NUM_LEGS],
        float theta2_out[NUM_LEGS],
        float theta3_out[NUM_LEGS],
        const float& a, const float& b, const float& c);
bool inverseKinematics(
        float x, float y, float z,
        float& theta1,
        float& theta2,
        float& theta3,
        const float& a, const float& b, const float& c);
}