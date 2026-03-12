#include "RobotConfig.hpp"

float a = 60.0;
float b = 95.0;
float c = 180.0;

float xl[NUM_LEGS] = {  80,  80, -80, -80 };
float yl[NUM_LEGS] = {  60, -60,  60, -60 };
float zl[NUM_LEGS] = {   0,   0,   0,   0 };

int sideSign[NUM_LEGS] = { 1, -1, 1, -1 };
;

int servoPins[NUM_LEGS][3] = {
    {18, 19, 21},
    {12, 13, 14},
    {2,3,4},
    {22, 23, 25}
};

float legEndPos[NUM_LEGS][3];

void initRobotConfig(){
};

bool inv = false;