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

void autonomous() {}


void competition_initialize() {}

void opcontrol() {

    pros::Controller master(pros::E_CONTROLLER_MASTER);
	smoothDriver.setTurnPID(2.5, 0, 4.4);

    smoothDriver.turn(90, 127);

    
    pros::delay(500);

    
}
