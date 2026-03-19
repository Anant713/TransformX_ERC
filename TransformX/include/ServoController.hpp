#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>

void initServos();

// Direct ESP32 control (debugging)
void writeServos(float t1[], float t2[], float t3[]);

// PCA9685 driver control
void writeServosDriver(float t1[], float t2[], float t3[]);

#endif