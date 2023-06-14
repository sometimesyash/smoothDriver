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

    driveDist(distance, speedMax);

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


void smoothDriver::runPIDTune(double stepCount){

    double tP = 0.0;
    double tI = 0.0;
    double tD = 0.0;


    double tPrevError = 0.0;
    double tError = 0.0;

    bool keepTesting = true;

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    int oscillationCount = 0;
    int countTimeout = 0;

    double target = 800;
    double kTu = 0;
    int timeOf = 0;
    double finalPow = 0;

    master.print(1, 1, "Testing in progres. KP: %d", tP);

    while(keepTesting){

        

        double curr = m_leftMotorGroup.get_positions()[0] + m_rightMotorGroup.get_positions()[0];

        tError = target - curr;

        finalPow = tP * tError;
 

        
        setPower(finalPow);

        if((tError > 5 && tPrevError < -5 || tError < -5 && tPrevError > 5)){

            oscillationCount++;
            timeOf +=50;

            master.print(1, 1, "Oscillation Detected: %d", oscillationCount);

            countTimeout = 0;

        } else{
            countTimeout++;

            if(countTimeout > 12){
                
                countTimeout = 0;
                oscillationCount = 0;
                timeOf = 0;
                
                master.print(1, 1, "Oscillation Lost: %d", oscillationCount);
            } else {
                timeOf += 50;
            }
            

            if(tError < 100 && finalPow < 10){
                tP += stepCount;

                target+=300;
            } else if (finalPow < 10) {
                tP += stepCount;

            }

        }

        if(oscillationCount > 12){

            keepTesting = false;

        } else if(oscillationCount < 1){

            if(tError > 200 && finalPow < 10){
                target += 500;
            }
            
            
            
        }

        
        pros::delay(50);

        tPrevError = tError;
    }

    setPower(0);

    kTu = oscillationCount * timeOf/ 1000.0;
    tI = (0.5 * tP) / kTu;
    tD = (0.125 * tP) * kTu;


    

    master.print(1, 1, "Complete");
    pros::screen::print(TEXT_MEDIUM, 2, "Final kP: %f", tP);
    pros::screen::print(TEXT_MEDIUM, 3, "Final kI: %f", tI);
    pros::screen::print(TEXT_MEDIUM, 4, "Final kD: %f", tD);

    kP = tP;
    kI = tI;
    kD = tD;



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

void smoothDriver::setExitConditions(double threshold, double totalTime, double tThreshold){

    driveThreshold = threshold;
    longTime = totalTime;
    turnThreshold = tThreshold;

}


void smoothDriver::driveDist(double distance, double speedMax){


    double previousError = 0.0;
    double error = 0.0;
    double integral = 0.0;


    bool runPID = true;
    int counter = 0;
    double timecount = 0;

    m_leftMotorGroup.tare_position();
    m_rightMotorGroup.tare_position();



    while(runPID){


        double curr = m_leftMotorGroup.get_positions()[0] + m_rightMotorGroup.get_positions()[0];

        error = distance - curr;

        double prop = kP * error;
        integral = integral + kI * error;
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




        pros::delay(50);

    }

    setPower(0);



}


void smoothDriver::turn(int targetAng, double speedMax){
    
    double previousError = 0.0;
    double error = 0.0;
    double integral = 0.0;


    bool runTurnPID = true;
    int counter = 0;
    double timecount = 0;

    if(inertialIn == false){
        runPID = false;
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

void smoothDriver::runPIDTurnTune(double stepCount){

    double turnTp = 0.0;
    double turnTi = 0.0;
    double turnTd = 0.0;

    double tPrevError = 0.0;
    double tError = 0.0;

    bool keepTurning = true;

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    int turnOsC = 0;

    int turnTimeout = 0;
    double turnInt = 0.0;

    double target = 90 + m_imu.get_heading();
    double ktTu = 0.0;
    int timeTurn = 0;
    double finalTurnPow = 0;

    master.print(1, 1, "Turn Test");

    while(keepTurning){

        tError = target - m_imu.get_heading();

        if(tError > 180.0){

            tError -= 360.0;

        } else if (tError < -180.0) {

            tError+=360;
        }

        finalTurnPow = turnTp * tError;

        setTurnPower(finalTurnPow);

         if((tError > 5 && tPrevError < -5 || tError < -5 && tPrevError > 5)){

            turnOsC++;
            timeTurn +=50;

            master.print(1, 1, "Oscillation Detected: %d", turnOsC);

            turnTimeout = 0;

        } else {

            turnTimeout++;

            if(turnTimeout > 12){
                turnTimeout = 0;
                turnOsC = 0;
                timeTurn = 0;

                master.print(1, 1, "Oscillation Lost");
            }

            if(tError < 15 && finalTurnPow < 10){
                turnTp += stepCount;

                target += 90;
            } else if(finalTurnPow < 10){
                turnTp += stepCount;

            }

        }

        if(turnOsC > 12){

            keepTurning = false;

        } else if(turnOsC < 1){

            if(tError > 20 && finalTurnPow < 10){

                target += 90;

            }
            tPrevError = tError;
            pros::delay(50);
            
        }

    }

    setTurnPower(0);

    master.print(1, 1, "Final kP: %d", turnTp);

}
