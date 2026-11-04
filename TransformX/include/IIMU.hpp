#pragma once

/**
 * @brief Abstract IMU interface
 *
 * Allows Robot class to remain independent
 * from specific IMU hardware.
 */
class IIMU
{
public:
    virtual ~IIMU() = default;

    /**
     * @brief Initialize IMU hardware
     */
    virtual bool init() = 0;

    /**
     * @brief Read latest IMU values
     */
    virtual void update() = 0;

    /**
     * @brief Get roll angle in degrees
     */
    virtual float getRoll() const = 0;

    /**
     * @brief Get pitch angle in degrees
     */
    virtual float getPitch() const = 0;

    /**
     * @brief Get yaw angle in degrees
     */
    virtual float getYaw() const = 0;

    /**
     * @brief Log IMU data with timestamp
     */
    virtual void logData() const = 0;
};