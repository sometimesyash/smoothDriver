# **Smooth Driver**

The intuitive library that allows for you to get started in as little as 2 lines of code

Version: 0.5.0

## Installation

1. Download the template, extract it and open it in PROS.

2. In `src/main.cpp`, at the top, declare your motors, either 4 or 6 as per your drive configeration, using `pros::Motor motorName (motorPort, motorGearset, direction, E_MOTOR_ENCODER_DEGREES)`. Set each motor by replacing motorPort, motorGearset etc.

3. Initialize the smoothDriver using `smoothDriver smoothDriver(inertialPort, leftBack, rightBack, leftMid, rightMid, leftFront, rightFront);` , replacing the variables with your corresponding motors. Note, if you are using a 4m drive, you the format is the same minus the leftMid and rightMid motors.

4. In the `Intialise()` function, reset the robot using `smoothDriver.resetDrive()`
5. You are ready to start smooth driving!

## Driver Control

Currently, smoothDriver only supports tank drive controls. The smoothDriver uses a double-S curve drive for its control, with two variables that are able to be adjusted according to user preference. You can find the mathematics of the positive curve [here.](https://www.desmos.com/calculator/oldghlasqy) 
The negative version is a reflection of this in the negative quadrant. 

1. _Optional:_ Tune your k and a values according to the curve
