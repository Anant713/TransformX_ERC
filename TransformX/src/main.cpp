#include "Robot.hpp"
#include "PCA9685Driver.hpp"
#include "Communication.hpp"

PCA9685Driver driver;
Robot robot(&driver);

void setup()
{
    Serial.begin(115200);
    driver.init();
}

void loop()
{
    Command cmd = receiveCommand();

    switch (cmd.type)
    {
    case CMD_HOME:
        robot.setPosition(0, 0, -20);
        break;

    case CMD_SET_POS:
        robot.setPosition(cmd.x, cmd.y, cmd.z);
        break;

    case CMD_SET_ANG:
        robot.setAngles(cmd.leg, cmd.t1, cmd.t2, cmd.t3);
        break;

    case CMD_WALK:
        // future gait
        break;
    }

    robot.update();
}