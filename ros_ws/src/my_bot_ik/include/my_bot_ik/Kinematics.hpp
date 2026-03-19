#pragma once

#include <array>

namespace my_bot_ik
{
bool inverseKinematics(
    double x, double y, double z,
    double a, double b, double c,
    double &theta1, double &theta2, double &theta3);

bool inverseKinematicsGlobal(
    const std::array<std::array<double, 3>, 4> &legEndPos,
    const std::array<double, 4> &xl,
    const std::array<double, 4> &yl,
    const std::array<double, 4> &zl,
    const std::array<double, 4> &sideSign,
    double a, double b, double c,
    std::array<double, 4> &theta1_out,
    std::array<double, 4> &theta2_out,
    std::array<double, 4> &theta3_out);
}