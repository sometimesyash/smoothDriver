#include "botconfig.h"
#include "pros/misc.h"
#include <cmath>



void smoothDriver::mapperWrapper(void *param){

    smoothDriver* driver = static_cast<smoothDriver*>(param);
    driver->map();

}

void smoothDriver::map(){


    double lastHeading = 0.0;
    double lastDistL = 0.0;
    double lastDistR = 0.0;



    while(runMap) {

        double leftEnc = m_leftMotorGroup[0].get_position();
        double rightEnc = m_rightMotorGroup[0].get_position();
        double theta = m_imu.get_heading();

        double changeL = m_leftMotorGroup[0].get_position() - lastDistL;
        double changeR = m_rightMotorGroup[0].get_position() - lastDistR;

        double avg = (changeL + changeR)/2;
        
        lastDistL = leftEnc;
        lastDistR = rightEnc;
        posX += avg * cos(theta);
        posY = avg * sin(theta);
    }

}