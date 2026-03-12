#include <ESP32Servo.h>

Servo servo1;

const int servoPin = 18;

void setup()
{
    Serial.begin(115200);

    servo1.attach(servoPin, 500, 2500);

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