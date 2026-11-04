#include <ESP32Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int servoPin1 = 18;
const int servoPin2 = 19;
const int servoPin3 = 20;
const int servoPin4 = 21;


void setup()
{
    Serial.begin(115200);

    servo1.attach(servoPin1, 500, 2500);
    servo2.attach(servoPin2, 500, 2500);
    servo3.attach(servoPin3, 1000, 2000);
    servo4.attach(servoPin4, 1000, 2000);

    Serial.println("Enter servo angle (0-180):");
}

void loop()
{
    if (Serial.available() > 0)
    {
        int angle = Serial.parseInt();

        if (angle >= 0 && angle <= 180)
        {
            servo1.write(angle);
            servo2.write(angle);
            servo3.write(angle);
            servo4.write(angle);

            Serial.print("Moving servo to: ");
            Serial.println(angle);
        }
        else
        {
            Serial.println("Invalid angle. Enter 0-180.");
        }

        while (Serial.available()) Serial.read(); 
    }
}