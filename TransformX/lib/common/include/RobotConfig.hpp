#pragma once

constexpr int NUM_LEGS = 4;
constexpr int NUM_JOINTS = 3;

// -------- Link lengths (mm) --------
extern float a;
extern float b;
extern float c;

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