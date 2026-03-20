#pragma once

#include "RobotConfig.hpp"


extern float theta1_out[NUM_LEGS];
extern float theta2_out[NUM_LEGS];
extern float theta3_out[NUM_LEGS];
// -------- Leg base positions in bot frame --------
extern float xl[NUM_LEGS];
extern float yl[NUM_LEGS];
extern float zl[NUM_LEGS];
// -------- Side sign (+1 right, -1 left) --------
extern int sideSign[NUM_LEGS];

// -------- Global desired leg positions --------
extern float legEndPos[NUM_LEGS][3];

// -------- Servo angle sign and offset mapping --------
// [servo_cmd] = [ik_angle]*[sign] + [offset]
extern float servoAngSigns[NUM_LEGS][NUM_JOINTS];
extern float servoOffsets[NUM_LEGS][NUM_JOINTS];

// -------- Function --------
void initRobotConfig();