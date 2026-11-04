#include "MPU6050Driver.hpp"

#include <Arduino.h>
#include <Wire.h>

MPU6050Driver::MPU6050Driver()
    : roll(0.0f),
      pitch(0.0f),
      yaw(0.0f),
      timestamp(0)
{
}

bool MPU6050Driver::init()
{
    Wire.begin();

    mpu.initialize();

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        return false;
    }

    Serial.println("MPU6050 connected.");

    return true;
}

void MPU6050Driver::update()
{
    mpu.getMotion6(&ax, &ay, &az,
                   &gx, &gy, &gz);

    timestamp = millis();

    // Simple accel-only estimation
    roll = atan2((float)ay, (float)az) * 180.0f / PI;

    pitch = atan2(
                 -(float)ax,
                 sqrt((float)ay * ay + (float)az * az))
             * 180.0f / PI;

    // No magnetometer → yaw unavailable
    yaw = 0.0f;
}

float MPU6050Driver::getRoll() const
{
    return roll;
}

float MPU6050Driver::getPitch() const
{
    return pitch;
}

float MPU6050Driver::getYaw() const
{
    return yaw;
}

void MPU6050Driver::logData() const
{
    Serial.print(timestamp);

    Serial.print(",");

    Serial.print(roll);

    Serial.print(",");

    Serial.print(pitch);

    Serial.print(",");

    Serial.println(yaw);
}