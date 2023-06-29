# **Smooth Driver**

The intuitive library that allows for you to get started in as little as 3 lines of code

Version: 0.5.0

## Installation

1. Download the template, extract it and open it in PROS.

2. In `src/main.cpp`, at the top, declare your motors, either 4 or 6 as per your drive configeration, using `pros::Motor motorName (motorPort, motorGearset, direction, E_MOTOR_ENCODER_DEGREES)`. Set each motor by replacing motorPort, motorGearset etc.

3. Initialize the smoothDriver using `smoothDriver smoothDriver(inertialPort, leftBack, rightBack, leftMid, rightMid, leftFront, rightFront);` , replacing the variables with your corresponding motors. Note, if you are using a 4m drive, you the format is the same minus the leftMid and rightMid motors.

4. In the `Intialise()` function, reset the robot using `smoothDriver.resetDrive()` - note if your called inertial sensor isn't plugged in, it will flash a warning to your controller, and you will not be able to call upon any turns in autonomous.
5. You are ready to start smooth driving!

## Driver Control

Currently, smoothDriver only supports tank drive controls. The smoothDriver uses a double-S curve drive for its control, with two variables that are able to be adjusted according to user preference. You can find the mathematics of the positive curve [here.](https://www.desmos.com/calculator/oldghlasqy) 
The negative version is a reflection of this in the negative quadrant.

Note you do not need to tune your active braking or curve K and A values, and can instead skip to starting the driver control.

### K and A values
The curve above has two indicators, the K value and the A value. The K value acts as the gradient, and shouldn't be less than 0. The A value defines the gradient of the tail of the curve - if you increase K, you need to increase A to ensure your robot doesn't stop driving at certain positions. You do not need to adjust this, but may wish to dependant on your drive curve.

Tune your k and a values according to the curve, using `smoothDriver.setKcurve(newK)` and  `smoothDriver.setAcurve(newA)`, replacing newK and newA with a double of your choice. The default is 0.1 and 0.4 respectively.

### Active Braking PD Lock

While in driver control mode, the robot can be designed to engage in an active brake lock, using a P and D value for the system. The purpose of this is to push back against any robot that may be pushing at you. You may need to tune these, although since it is in braking, you do not need it to be too accurate. If not set, the robot will not active brake.

1. Enable your braking using the function `smoothDriver.enableBraking(brakeP, brakeD, button)
2. The Button option is a pros digital button input, and will take any pros::digital values - e.g. `pros::E_CONTROLLER_DIGITAL_Y`
3. Once enabled, if you wish to disable, call `smoothDriver.disableBraking()`

### Start Drive

Within the `while(true)` loop in main.cpp, call `smoothDriver.startDriver()`. Your robot is now ready to drive! You can add in additional functions into your control below this, within the while loop.


## Autonomous

The autonomous in the Smooth Driver comes with a PID control loop, with a tracking algorithm that will continue to adjust your variables for the smoothest PID. The system also makes use of odometry, as well as self tuning PID functions that can be run on surfaces to produce starting values for your kP, kI and kD. Never tune your constants again!

### Basic Control

To use the PID controller against your own constants:
