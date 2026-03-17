#include "RobotConfig.hpp"

float a = 60.0;
float b = 95.0;
float c = 180.0;

float xl[NUM_LEGS] = {  0, 0, 0, 0 };
float yl[NUM_LEGS] = {  0, 0, 0, 0 };
float zl[NUM_LEGS] = {  0, 0, 0, 0 };

int sideSign[NUM_LEGS] = { 1, -1, 1, -1 };
;
// -------- Used only if using direct servo control thorugh ESP ------------
int servoPins[NUM_LEGS][3] = {
    {18, 19, 21},
    {12, 13, 14},
    {2,3,4},
    {22, 23, 25}
};
//----------- Servo offsets and directions matrices ([Angle from IK]*[Direction/sign matrix]+[Offset matrix]) ------------------
float servoAngSigns[NUM_LEGS][3] = {
    {-1,-1,1},
    {0,0,0},
    {0,0,0},
    {0,0,0}
};
float servoOffsets[NUM_LEGS][3] = {
    {90,90,90},
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

float legEndPos[NUM_LEGS][3];

void initRobotConfig(){
};

bool inv = false;

// float angOffsets[NUM_LEGS][3];
// angOffsets[0] = [90,90,]
