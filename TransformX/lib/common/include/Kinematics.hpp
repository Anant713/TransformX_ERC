#pragma once

#include "RobotConfig.hpp"

extern float theta1_out[NUM_LEGS];
extern float theta2_out[NUM_LEGS];
extern float theta3_out[NUM_LEGS];

bool inv_ken_global(float legEndPos[NUM_LEGS][NUM_JOINTS]);