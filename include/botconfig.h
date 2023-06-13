#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "main.h"

class smoothDriver {
    public:
        smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftMid, pros::Motor& rightMid, pros::Motor& leftFront, pros::Motor& rightFront);

        smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftFront, pros::Motor& rightFront);

        void drive(double pointX, double pointY, double speedMax);
        void driveDist(double distance, double speedMax, double P);

        void turnToFace(double pointX, double pointY, double speedMax);

        void turn(int targetAng, double speedMax, double p);


        void straightTune();

        void turnTune();

        void resetDrive();

        void setDrivePID(double newP, double newI, double newD);
        void setTurnPID(double newHP, double newHI, double newHD);

        void setExitConditions(double threshold, double totalTime, double tThreshold);


        void setKcurve(double newK);
        void setAcurve(double newA);
        void startDriver();
        double returnPos();

        double returnDist(int x, int y);
        void runPIDTune(double stepCount);
        void runPIDTurnTune(double stepCount);
        







    private:

    pros::MotorGroup m_leftMotorGroup;
    pros::MotorGroup m_rightMotorGroup;
    pros::Imu m_imu;

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double driveThreshold = 200;

    double localkP = 0;
    double localkI = 0;
    double localkD = 0;


    double turnkP = 0;
    double turnkI = 0;
    double turnkD = 0;
    double turnThreshold = 2;

    void setPower(double pow);
    void setTurnPower(double pow);

    int curveDrive(int inputVal);

    void map(void* ignore);


    double posX;
    double posY;

    double kCurve = 0.1;
    double aCurve = 0.4;
    double longTime = 5000;
    bool inertialIn = true;
    bool runMap = true;

    

};



