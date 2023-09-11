#include "botconfig.h"
#include "pros/misc.h"
#include <cmath>
#include <cstdint>

smoothDriver::smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftMid, pros::Motor& rightMid, pros::Motor& leftFront, pros::Motor& rightFront):
    m_leftMotorGroup({leftBack, leftMid, leftFront}),
    m_rightMotorGroup({rightBack, rightMid, rightFront}),
    m_imu(inertialPort),
    leftEnc('A', 'B'),
    rightEnc('c', 'd')
    {encodersIn = false;}

smoothDriver::smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftFront, pros::Motor& rightFront):
    m_leftMotorGroup({leftBack, leftFront}),
    m_rightMotorGroup({rightBack, rightFront}),
    m_imu(inertialPort),
    leftEnc('A', 'B'),
    rightEnc('c', 'd')
    {encodersIn = false;}

smoothDriver::smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftFront, pros::Motor& rightFront, std::uint8_t LEFT_TOP, std::uint8_t LEFT_BOTTOM, std::uint8_t RIGHT_TOP, std::uint8_t RIGHT_BOTTOM):
    m_leftMotorGroup({leftBack, leftFront}),
    m_rightMotorGroup({rightBack, rightFront}),
    m_imu(inertialPort),
    leftEnc(LEFT_TOP, LEFT_BOTTOM),
    rightEnc(RIGHT_TOP, RIGHT_BOTTOM)
    {encodersIn = true;}

smoothDriver::smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftMid, pros::Motor& rightMid, pros::Motor& leftFront, pros::Motor& rightFront, std::uint8_t LEFT_TOP, std::uint8_t LEFT_BOTTOM, std::uint8_t RIGHT_TOP, std::uint8_t RIGHT_BOTTOM):
    m_leftMotorGroup({leftBack, leftMid, leftFront}),
    m_rightMotorGroup({rightBack, rightMid, rightFront}),
    m_imu(inertialPort),
    leftEnc(LEFT_TOP, LEFT_BOTTOM),
    rightEnc(RIGHT_TOP, RIGHT_BOTTOM)
    {encodersIn = true;}




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
            
            //Double checks if Braking, if Not, curves the input.
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
            //Brake Function
            if(master.get_digital(brakeButton)){
                curBreak = true; //Disables Input
                double leftSide = m_leftMotorGroup[0].get_position();
                double rightSide = m_rightMotorGroup[0].get_position(); 
                //Gets current position and brakes.


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
        //If Indicated, set the brake aim to the current position
        
        targetBrakeL = in1;
        targetBrakeR = in2;
    } 

    m_leftMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    m_rightMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    //Setting brake mode to PROS BRAKE

    double targetL = targetBrakeL;
    double targetR = targetBrakeR;
    double currL = m_leftMotorGroup[0].get_position();
    double currR = m_rightMotorGroup[0].get_position();
    //Current Position of the robot

    double errorL = targetL - currL;
    double errorR = targetR - currR;
    //Error between the current and the position
    
    double devL = errorL - brakePrevL;
    double devR = errorR - brakePrevR;
    //derivative value

    double finalLeftPow = (brakeP * errorL) + (brakeD * devL); //Calculation of power using error * KP + derivative * KD
    double finalRightPow = (brakeP * errorL) + (brakeD * devR);

    if(errorL > 50 || errorL < -50){
        m_leftMotorGroup.move(finalLeftPow);
    } else {
        m_leftMotorGroup.move(0);
    } //Bands for a little adjustment

    if(errorR > 50 || errorR < -50){
        m_rightMotorGroup.move(finalRightPow);
    } else {
        m_rightMotorGroup.move(0);
    }

    brakePrevL = errorL;
    brakePrevR = errorR;
    m_leftMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    m_rightMotorGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    //Reset to Coast
}

int smoothDriver::curveDrive(int inputVal){
    //Define Variables
    int speed;
    int x;
    
    if(inputVal >= 0){
        
        x = inputVal;
        speed = pow((1/(1+exp(-kCurve*(x-96)))), aCurve) *127; 
    } else if(inputVal <0){
        //Flips the X value
        x = -inputVal;
        speed = pow((1/(1+exp(-kCurve*(x-96)))), aCurve) *(-127); 
    }
    

    if(inputVal < 0){
        speed = -speed;
    }
    return speed;
}


void smoothDriver::resetDrive(){ 
    //In order to tare and reset the sensors on the vehicle 

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    m_leftMotorGroup.tare_position();
    m_rightMotorGroup.tare_position();
    //Resets position to 0

    if(m_imu.get_heading() == PROS_ERR_F){
        
        //Detects PROS_ERR_F --> faulty or not connected IMU
        
        master.print(1, 1, "IMU Not Connected"); //Message to Controller
        inertialIn = false;

    } else {

        m_imu.reset(); //Calibrates the IMU

        while(m_imu.is_calibrating()){
            master.print(1, 1, "SmoothDriver Calibrating");
            inertialIn = true;
        }

        posX = 0; 
        posY = 0;
        master.print(1, 1, "Ready To Drive"); //Confirmation all is calibrated and tared
    }
    
}
