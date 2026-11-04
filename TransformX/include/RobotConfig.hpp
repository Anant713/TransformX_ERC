#pragma once

#include <stdint.h>

constexpr int NUM_LEGS = 4;
constexpr int NUM_JOINTS = 3;

/**
 * @brief Physical geometry (meters or mm)
 * Indexing convention for legs: 0 - Front Left, 1 - Back Left, 2 - Front Right, 3 - Back Right
 */
namespace RobotConfig
{
    // Shoulder positions in body frame
    constexpr float xl[NUM_LEGS] = {0.0f, 0.0f, 0.0f, 0.0f};
    constexpr float yl[NUM_LEGS] = {0.0f, 0.0f, 0.0f, 0.0f};
    constexpr float zl[NUM_LEGS] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Link lengths
    constexpr float a = 60.0f;
    constexpr float b = 95.0f;
    constexpr float c = 180.0f;

    // Servo mapping
    constexpr int servoOffsets[NUM_LEGS][NUM_JOINTS] = {
        {90, 90, 90},
        {90, 90, 90},
        {90, 90, 90},
        {90, 90, 90}};

    constexpr int servoSigns[NUM_LEGS][NUM_JOINTS] = {
        {-1, -1, 1},
        {-1, 1, -1},
        {1, 1, -1},
        {1, -1, 1}};
}