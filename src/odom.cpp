#include "botconfig.h"
#include "pros/misc.h"
#include <cmath>



void smoothDriver::mapperWrapper(void *param){

    smoothDriver* driver = static_cast<smoothDriver*>(param);
    driver->map();

}

double smoothDriver::getLeftDist(){

    double leftVal;
    if(encodersIn){
        leftVal = leftEnc.get_value();
    } else {
        leftVal = m_leftMotorGroup[0].get_position();
    }


    return leftVal;
    
}

double smoothDriver::getRightDist(){

    double rightVal;
    if(encodersIn){
        rightVal = rightEnc.get_value();
    } else {
        rightVal = m_rightMotorGroup[0].get_position();
    }


    return rightVal;
    
}

double leftDist = 0;
double rightDist = 0;



void smoothDriver::map(){


    double lastHeading = 0.0;
    double lastDistL = 0.0;
    double lastDistR = 0.0;
    double lastTheta = 0.0;
    double theta = 0.0;
    bool Which = false;

    if(inertialIn){ Which = true; };



    while(runMap) {

        if(Which){
            double leftEnc = getLeftDist();
            double rightEnc = getRightDist();
            double theta = m_imu.get_heading();

            double changeL = leftEnc - lastDistL;
            double changeR =rightEnc - lastDistR;

            double avg = (changeL + changeR)/2;
            
            lastDistL = leftEnc;
            lastDistR = rightEnc;

            posX += avg * cos(theta);
            posY += avg * sin(theta);

        } else {
            double leftEnc = getLeftDist();
            double rightEnc = getRightDist();
            double changeL = leftEnc - lastDistL;
            double changeR = rightEnc - lastDistR;
            

            double changeTheta = (changeL - changeR)/(leftDist + rightDist);
            theta += changeTheta;

            double Y = 2*((changeR/changeTheta)+rightDist) * (theta + (changeTheta/2));

            posX+= Y * sin(theta + (changeTheta/2));
            posY += Y * cos(theta + (changeTheta/2));



        }




        

        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "XPos:%3d", posX);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Ypos: %3d", posY);
    }

}
