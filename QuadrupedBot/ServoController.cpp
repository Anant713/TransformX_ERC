#include <Arduino.h>
#include <ESP32Servo.h>
#include "RobotConfig.hpp"
#include "ServoController.hpp"

Servo servos[NUM_LEGS][3];

void initServos()
{
    // ESP32 requires PWM timer allocation
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    for (int i = 0; i < NUM_LEGS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            servos[i][j].setPeriodHertz(50);   // standard servo frequency
            servos[i][j].attach(servoPins[i][j], 500, 2500); // pulse width range
        }
    }
}

void writeServos(float t1[], float t2[], float t3[])
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        servos[i][0].write(t1[i]);
        servos[i][1].write(t2[i]);
        servos[i][2].write(t3[i]);
    }
}