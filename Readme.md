# QuadrupedBot

## Repository Structure

This repository contains the control system for a quadruped robot, organized into the following modules:

- **Communication**: Handles robot communication protocols
- **Kinematics**: Implements forward and inverse kinematics calculations
- **Servo Control**: Manages servo motor operations

## Key Files

- **QuadrupedBot.ino**: Main control file for the robot
- **RobotConfig.cpp / RobotConfig.hpp**: Define physical structure, servo motor states, and provide global access to configuration variables

## Commands

### Forward Kinematics
Set joint angles from top to bottom on the leg ( this is for front left leg, theta1 and theta2 are used as per convention in inverse kinematics i.e. measured from the previous leg):
```
setang <theta3> <theta1> <theta2>
```

### Inverse Kinematics
Set leg position relative to the robot center:
```
setpos <x> <y> <z>
```

### Homing
Reset the robot to home position:
```
home
```
