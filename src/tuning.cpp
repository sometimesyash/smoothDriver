#include "botconfig.h"
#include "pros/misc.h"
#include <cmath>


void smoothDriver::setExitConditions(double threshold, double totalTime, double tThreshold){

    driveThreshold = threshold;
    longTime = totalTime;
    turnThreshold = tThreshold;

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