#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <ESP32Servo.h>
#include "RobotConfig.hpp"

void initServos();
void writeServos(float t1[], float t2[], float t3[]);

#endif