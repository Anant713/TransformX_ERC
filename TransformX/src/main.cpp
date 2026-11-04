#include "Robot.hpp"
#include "PCA9685Driver.hpp"
#include "Communication.hpp"
#include "MPU6050Driver.hpp"

PCA9685Driver servoDriver;
MPU6050Driver mpu;
Robot robot(&servoDriver, &mpu);

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10000); // Set timeout for Serial read operations
    servoDriver.init();
    mpu.init();
}

void loop()
{
    Command cmd = receiveCommand();

    switch (cmd.type)
    {
    case CMD_HOME:
        robot.home(0,100.176,-247.176);// (0,127,18,-247.18) is home for (0,45,45) joint angles
        robot.printAngleState();
        break;

    case CMD_SET_POS:
        robot.setPosition(cmd.leg, cmd.x, cmd.y, cmd.z);
        robot.printAngleState();
        break;

    case CMD_SET_ANG:
        robot.setAngles(cmd.leg, cmd.t1, cmd.t2, cmd.t3); 
        break;
    case CMD_WALK:
        robot.walk();// future gait
        break;

    default:
        robot.idle();
        break;
    }

    robot.update();
    robot.idle(); // After executing the command, go back to idle until next command is received

}