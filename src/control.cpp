#include "botconfig.h"
#include "pros/misc.h"
#include <cmath>

//Control Algorithms for Autonomous Control


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


void smoothDriver::driveAltDist(double distance, double speedMax, double P){


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
