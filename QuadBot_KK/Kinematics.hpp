#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

// Run full IK pipeline. legIndex=-1 for all legs, 0-3 for single leg.
bool inv_ken_global(float legEndPos[4][3], bool &inv, int legIndex = -1);

// Run IK and print angles WITHOUT moving servos
void testIK(float x, float y, float z, int legIndex);

// Raw IK solver for one leg
bool inverseKinematics(float &x, float &y, float &z,
                       float &theta1,
                       float &theta2,
                       float &theta3);

extern float theta1_out[4];
extern float theta2_out[4];
extern float theta3_out[4];

#endif
