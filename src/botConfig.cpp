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

    
        if(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)){
            m_rightMotorGroup.move(curveDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
        } else {
            m_rightMotorGroup.move(0);
        }

        if(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)){
            m_leftMotorGroup.move(curveDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
        } else {
            m_leftMotorGroup.move(0);
        }
    

}

double smoothDriver::returnDist(int x, int y){
    return std::sqrt(pow((x - posX), 2) + pow((y-posY), 2));
}

void smoothDriver::drive(double pointX, double pointY, double speedMax){

    double distance = returnDist(pointX, pointY);

    driveDist(distance, speedMax, 0.1);

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
    }
    
}


void smoothDriver::driveDist(double distance, double speedMax, double P){


    double previousError = 0.0;
    double error = 0.0;
    double integral = 0.0;


    bool runPID = true;
    int counter = 0;
    double timecount = 0;

    double startkP = kP;
    double startkI = kI;
    double startkD = kD;

    double theta[3] = {kP, kI, kD};

    m_leftMotorGroup.tare_position();
    m_rightMotorGroup.tare_position();



    while(runPID){


        double curr = m_leftMotorGroup.get_positions()[0] + m_rightMotorGroup.get_positions()[0];

        error = distance - curr;

        double prop = kP * error;
        integral = integral + (kI * error);
        double derivative = kD * (error - previousError);

        double finalPow = prop + integral + derivative;

        if(finalPow > speedMax){
            finalPow = speedMax;
        } else if(finalPow < -speedMax){
            finalPow = -speedMax;
        }

        

        setPower(finalPow);
        previousError = error;

        if(std::abs(error) < driveThreshold){
            counter += 50;
        } else {
            counter = 0;
        }

        if(counter >= 200 || timecount >= longTime){
            runPID = false;
        }

        timecount+= 50;

        double x[3] = {error, integral, derivative};

        double gamma = P * x[0] / (1.0 + x[0] * P * x[0]);

        for(int i = 0; i<3; i++){
            theta[i] += gamma * error * x[i];
        }

        kP = theta[0];
        kI = theta[1];
        kD = theta[2];

        double P_temp = P;
        P = P_temp - gamma * x[0] * P_temp;


        pros::delay(50);

    }

    kP = startkP;
    kI = startkI;
    kD = startkD;

    setPower(0);



}


void smoothDriver::turn(int targetAng, double speedMax, double P){
    
    double previousError = 0.0;
    double error = 0.0;
    double integral = 0.0;

     pros::Controller master(pros::E_CONTROLLER_MASTER);


    bool runTurnPID = true;
    int counter = 0;
    double timecount = 0;

    if(inertialIn == false){
        runTurnPID = false;
        master.print(1, 1, "IMU Not Connected");
    }

    while(runTurnPID){

        error = targetAng - m_imu.get_heading();

        if(error > 180.0) {
            error -= 360.0;
        } else if(error < -180.0){
            error+= 360.0;
        }

        integral += error;
        double derivative = error - previousError;
        
        double finalPow = (integral * turnkI) + (derivative * turnkD) + (turnkP * error);

        if(finalPow > speedMax){
            finalPow = speedMax;
        } else if(finalPow < -speedMax){
            finalPow = -speedMax;
        }

        
        setTurnPower(finalPow);

        if(std::abs(error) < turnThreshold){
            counter += 50;
        } else {
            counter = 0;
        }

        if(counter >= 200 || timecount >= longTime){
            runTurnPID = false;
        }

        timecount+= 50;

        pros::delay(50);

    }

    setTurnPower(0);
}
