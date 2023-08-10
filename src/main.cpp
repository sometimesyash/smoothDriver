#include "opcontrol.h"
#include "main.h"
#include <cmath>
#include "botConfig.h"

#include "pros/motors.hpp"


//Smooth Driver CODE

 pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::Motor leftBack(8, true);
    pros::Motor rightBack(18);
    pros::Motor leftMid(9, true);
    pros::Motor rightMid(16);
    pros::Motor leftFront(17, true);
    pros::Motor rightFront(20);
    
    smoothDriver smoothDriver(2, leftBack, rightBack, leftMid, rightMid, leftFront, rightFront);




void initialize() {
	pros::lcd::initialize();

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::delay(500);

    smoothDriver.resetDrive();

}


void disabled() {}

void autonomous() {

    smoothDriver.runPIDTune(0.1);
    pros::delay(500);
    smoothDriver.driveDist(1000, 80);

   /* pros::delay(1000);
    smoothDriver.driveAltDist(500, 127);*/
}


void competition_initialize() {}

void opcontrol() {

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    smoothDriver.enableBraking(1.0, 0.8, pros::E_CONTROLLER_DIGITAL_A);
    
    while(true){
        smoothDriver.startDriver();
    }

    
    pros::delay(500);

    
}
