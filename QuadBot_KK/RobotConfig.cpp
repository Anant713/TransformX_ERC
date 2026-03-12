#include "RobotConfig.hpp"

float a = 60.0;
float b = 95.0;
float c = 180.0;

float xl[NUM_LEGS] = {  80,  80, -80, -80 };
float yl[NUM_LEGS] = {  60, -60,  60, -60 };
float zl[NUM_LEGS] = {   0,   0,   0,   0 };

int sideSign[NUM_LEGS] = { 1, -1, 1, -1 };

int servoPins[NUM_LEGS][3] = {
    {18, 19, 21},
    {12, 13, 14},
    { 2,  3,  4},
    {22, 23, 25}
};

float legEndPos[NUM_LEGS][3];

// Default home position: directly below hip, legs extended down
// Tune this live using "sethome x y z" command
float homePos[3] = { 0.0, 0.0, -200.0 };

// Which leg is active in single-leg debug mode (0-3)
int activeLeg = 0;

bool inv = false;

void initRobotConfig() {
    // Initialise all legs to home position
    for (int i = 0; i < NUM_LEGS; i++) {
        legEndPos[i][0] = homePos[0];
        legEndPos[i][1] = homePos[1];
        legEndPos[i][2] = homePos[2];
    }
}
