#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

bool inverseKinematics(float &x, float &y, float &z,
                       float &theta1,
                       float &theta2,
                       float &theta3);

bool inv_ken_global(float legEndPos[4][3], bool &inv);// Leg end pos of the frontmost left leg wrt to bot centre

extern float theta1_out[4];
extern float theta2_out[4];
extern float theta3_out[4];

#endif