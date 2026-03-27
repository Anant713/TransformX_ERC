#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#define NUM_LEGS 4

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

// -------- Servo pins [leg][joint] --------
extern int servoPins[NUM_LEGS][3];

// -------- Global desired leg positions --------
extern float legEndPos[NUM_LEGS][3];

// -------- Tunable home position (x,y,z relative to hip) --------
extern float homePos[3];

// -------- Active leg for single-leg debug mode --------
extern int activeLeg;

// -------- Function --------
void initRobotConfig();

// ------------ Flag if command is for forward or inverse kinematics -----------
extern bool inv;

#endif
