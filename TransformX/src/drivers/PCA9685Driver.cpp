#include "PCA9685Driver.hpp"
#include <Wire.h>

uint16_t PCA9685Driver::angleToPulse(float angle)
{
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void PCA9685Driver::init()
{
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
}

void PCA9685Driver::writeAll(float t1[], float t2[], float t3[])
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        int ch0 = i * 3;
        int ch1 = i * 3 + 1;
        int ch2 = i * 3 + 2;

        pwm.setPWM(ch0, 0, angleToPulse(t1[i]));
        pwm.setPWM(ch1, 0, angleToPulse(t2[i]));
        pwm.setPWM(ch2, 0, angleToPulse(t3[i]));
    }
}