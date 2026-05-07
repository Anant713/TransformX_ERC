#include "Robot.hpp"
#include "PCA9685Driver.hpp"
#include "Communication.hpp"

PCA9685Driver driver;
Robot robot(&driver);

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10000); // Set timeout for Serial read operations
    driver.init();
}

void loop()
{
    Command cmd = receiveCommand();

    switch (cmd.type)
    {
    case CMD_HOME:
        robot.home(0,0,-20);
        robot.printAngleState();
        break;

    case CMD_SET_POS:
        robot.setPosition(cmd.leg, cmd.x, cmd.y, cmd.z);
        
        break;

    case CMD_SET_ANG:
        robot.setAngles(cmd.leg, cmd.t1, cmd.t2, cmd.t3);
        robot.printAngleState();
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