#include <Arduino.h>

#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>

#include "RobotConfig.hpp"
#include "ServoController.hpp"

// -------- Direct ESP32 Servo Control --------
Servo servos[NUM_LEGS][3];

// -------- PCA9685 Driver --------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse limits for PCA9685
#define SERVO_MIN 100
#define SERVO_MAX 500
#define SERVO_FREQ  50

// Convert angle → PCA9685 pulse
uint16_t angleToPulse(float angle)
{
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void initServos()
{
    // -------- Direct servo control setup --------
    // ESP32PWM::allocateTimer(0);

    // for (int i = 0; i < NUM_LEGS; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         servos[i][j].setPeriodHertz(50);
    //         servos[i][j].attach(servoPins[i][j], 500, 2500);
    //     }
    // }

    // -------- PCA9685 setup --------


    // Explicit I2C init (IMPORTANT)
    //Wire.begin(SDA_PIN, SCL_PIN);
    //Wire.setClock(100000);   // stable for debugging
    Wire.begin();
 
    pwm.begin();
    //pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ); // 50Hz servo frequency
    delay(10);
}

// void writeServos(float t1[], float t2[], float t3[])
// {
//     for (int i = 0; i < NUM_LEGS; i++)
//     {
//         servos[i][0].write(t1[i]);
//         servos[i][1].write(t2[i]);
//         servos[i][2].write(t3[i]);
//     }
// }

void writeServosDriver(float t1[], float t2[], float t3[])
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        int ch0 = i*3;
        int ch1 = i*3 + 1;
        int ch2 = i*3 + 2;

        pwm.setPWM(ch0, 0, angleToPulse(t1[i]));
        pwm.setPWM(ch1, 0, angleToPulse(t2[i]));
        pwm.setPWM(ch2, 0, angleToPulse(t3[i]));
        //pwm.setPWM(0, 0, 300);
    }
}