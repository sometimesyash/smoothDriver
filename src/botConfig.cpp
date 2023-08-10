#include "botconfig.h"
#include "pros/misc.h"
#include <cmath>

smoothDriver::smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftMid, pros::Motor& rightMid, pros::Motor& leftFront, pros::Motor& rightFront):
    m_leftMotorGroup({leftBack, leftMid, leftFront}),
    m_rightMotorGroup({rightBack, rightMid, rightFront}),
    m_imu(inertialPort)
    {}

smoothDriver::smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftFront, pros::Motor& rightFront):
    m_leftMotorGroup({leftBack, leftFront}),
    m_rightMotorGroup({rightBack, rightFront}),
    m_imu(inertialPort)
    {}



void smoothDriver::setDrivePID(double newP, double newI, double newD){
    kP = newP;
    kI = newI;
    kD = newD;

}

void smoothDriver::setTurnPID(double newHP, double newHI, double newHD){
    turnkP = newHP;
    turnkI = newHI;
    turnkD = newHD;
}

void smoothDriver::setPower(double pow){
    m_leftMotorGroup.move(pow);
    m_rightMotorGroup.move(pow);
}

void smoothDriver::setTurnPower(double pow){
    m_leftMotorGroup.move(pow);
    m_rightMotorGroup.move(-pow);
}

void smoothDriver::setKcurve(double newK){
    kCurve = newK;
}

void smoothDriver::setAcurve(double newA){
    aCurve = newA;
}

void smoothDriver::startDriver(){

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    
        if(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) && curBreak == false){
            m_rightMotorGroup.move(curveDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
        } else {
            m_rightMotorGroup.move(0);
        }

        if(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) && curBreak == false){
            m_leftMotorGroup.move(curveDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
        } else {
            m_leftMotorGroup.move(0);
        }

        if(braking){
            if(master.get_digital(brakeButton)){
                curBreak = true;
                double leftSide = m_leftMotorGroup[0].get_position();
                double rightSide = m_rightMotorGroup[0].get_position();


                brake(leftSide, rightSide);
                resetBrakes = false;
            } else { 
                curBreak = false;
                resetBrakes = true; 
            }
        }
        

}

void smoothDriver::enableBraking(double breakP, double breakD, pros::controller_digital_e_t button){

    brakeButton = button;
    brakeP = breakP;
    brakeD = breakD;

}

double smoothDriver::returnDist(int x, int y){
    return std::sqrt(pow((x - posX), 2) + pow((y-posY), 2));
}

void smoothDriver::drive(double pointX, double pointY, double speedMax){

    double distance = returnDist(pointX, pointY);

    driveDist(distance, speedMax);

}

void smoothDriver::brake(double in1, double in2){
    
    if(resetBrakes){
        
        targetBrakeL = in1;
        targetBrakeR = in2;
    }

    m_leftMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    m_rightMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    

    double targetL = targetBrakeL;
    double targetR = targetBrakeR;

    double currL = m_leftMotorGroup[0].get_position();
    double currR = m_rightMotorGroup[0].get_position();

    double errorL = targetL - currL;
    double errorR = targetR - currR;

    double devL = errorL - brakePrevL;
    double devR = errorR - brakePrevR;

    double finalLeftPow = (brakeP * errorL) + (brakeD * devL);
    double finalRightPow = (brakeP * errorL) + (brakeD * devR);

    if(errorL > 50 || errorL < -50){
        m_leftMotorGroup.move(finalLeftPow);
    } else {
        m_leftMotorGroup.move(0);
    }

    if(errorR > 50 || errorR < -50){
        m_rightMotorGroup.move(finalRightPow);
    } else {
        m_rightMotorGroup.move(0);
    }

    brakePrevL = errorL;
    brakePrevR = errorR;
    m_leftMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    m_rightMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    
}

int smoothDriver::curveDrive(int inputVal){
    int speed;
    int x;
    if(inputVal >= 0){
        x = inputVal;
    } else if(inputVal <0){
        x = -inputVal;
    }
    speed = pow((1/(1+exp(-kCurve*(x-96)))), aCurve) *127; 

    if(inputVal < 0){
        speed = -speed;
    }
    return speed;
}


void smoothDriver::resetDrive(){

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    m_leftMotorGroup.tare_position(); 
    m_rightMotorGroup.tare_position();
    

    if(m_imu.get_heading() == PROS_ERR_F){

        master.print(1, 1, "IMU Not Connected");
        inertialIn = false;

    } else {

        m_imu.reset();

        while(m_imu.is_calibrating()){
            master.print(1, 1, "SmoothDriver Calibrating");
            inertialIn = true;
        }

        master.print(1, 1, "Ready To Drive");
        pros::Task mapping(smoothDriver::mapperWrapper);
    }
    
}