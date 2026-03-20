#include "Kinematics.hpp"
#include "RobotConfig.hpp"
#include <cmath>


    bool inverseKinematics(
        float x, float y, float z,
        float a, float b, float c,
        float& theta1,
        float& theta2,
        float& theta3)
    {
        const float r = std::sqrt(x * x + y * y);

        const float D = ((r - a) * (r - a) + z * z - b * b - c * c) / (2.0f * b * c);

        if (D > 1.0f || D < -1.0f)
        {
            return false;
        }

        theta3 = std::atan2(std::sqrt(1.0f - D * D), D);

        theta2 = std::atan2(-z, r - a)
               - std::atan2(c * std::sin(theta3),
                            b + c * std::cos(theta3));

        theta1 = std::atan2(x, y);

        constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
        theta1 *= RAD_TO_DEG;
        theta2 *= RAD_TO_DEG;
        theta3 *= RAD_TO_DEG;

        return true;
    }

    bool inv_kin_global(
        const float legEndPos[NUM_LEGS][NUM_JOINTS],
        const float xl[NUM_LEGS],
        const float yl[NUM_LEGS],
        const float zl[NUM_LEGS],
        const int sideSign[NUM_LEGS],
        float theta1_out[NUM_LEGS],
        float theta2_out[NUM_LEGS],
        float theta3_out[NUM_LEGS])
    {
        for (int i = 0; i < NUM_LEGS; i++)
        {
            const float x_local = legEndPos[i][0] - xl[i];
            float y_local = legEndPos[i][1] - yl[i];
            const float z_local = legEndPos[i][2] - zl[i];

            y_local *= sideSign[i];

            const bool reachable = inverseKinematics(
                x_local, y_local, z_local,
                a, b, c,
                theta1_out[i], theta2_out[i], theta3_out[i]);

            if (!reachable)
            {
                return false;
            }

            theta1_out[i] *= sideSign[i];
        }

        return true;
    }