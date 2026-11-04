#pragma once

#include <Adafruit_PWMServoDriver.h>
#include "IServoDriver.hpp"
#include "RobotConfig.hpp"
#define SDA_PIN 21
#define SCL_PIN 22
#define SERVO_MIN 100
#define SERVO_MAX 500
#define SERVO_FREQ  50
/**
 * @brief PCA9685-based servo driver implementation
 */
class PCA9685Driver : public IServoDriver
{
private:
    Adafruit_PWMServoDriver pwm;

    /**
     * @brief Convert angle (deg) → PWM pulse
     */
    uint16_t angleToPulse(float angle);

public:
    void init() override;

    void writeAll(float t1[], float t2[], float t3[]) override;
};