#pragma once

/**
 * @brief Abstract interface for servo drivers
 *
 * This allows the Robot class to remain hardware-independent.
 * Any driver (PCA9685, direct PWM, simulation) must implement this.
 */
class IServoDriver
{
public:
    virtual ~IServoDriver() = default;

    /**
     * @brief Initialize hardware (I2C, PWM, etc.)
     */
    virtual void init() = 0;

    /**
     * @brief Write joint angles for all legs
     *
     * @param t1 Hip joint angles
     * @param t2 Knee joint angles
     * @param t3 Ankle joint angles
     */
    virtual void writeAll(float t1[], float t2[], float t3[]) = 0;
};