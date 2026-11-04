#pragma once

#include <MPU6050.h>
#include "IIMU.hpp"

class MPU6050Driver : public IIMU
{
private:
    MPU6050 mpu;

    float roll;
    float pitch;
    float yaw;

    unsigned long timestamp;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

public:
    MPU6050Driver();

    bool init() override;

    void update() override;

    float getRoll() const override;
    float getPitch() const override;
    float getYaw() const override;

    void logData() const override;
};