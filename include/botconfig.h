#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "main.h"

class smoothDriver {
    public:
        smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftMid, pros::Motor& rightMid, pros::Motor& leftFront, pros::Motor& rightFront);

        smoothDriver(int inertialPort, pros::Motor& leftBack, pros::Motor& rightBack, pros::Motor& leftFront, pros::Motor& rightFront);

        void drive(double pointX, double pointY, double speedMax);
        void driveDist(double distance, double speedMax);
        void driveAltDist(double distance, double speedMax, double P);


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
        void enableBraking(double breakP, double breakD, pros::controller_digital_e_t button);
        void startDriver();
        double returnPos();

        double returnDist(int x, int y);
        void runPIDTune(double stepCount);
        void runPIDTurnTune(double stepCount);
        
        static void mapperWrapper(void* param);
        void map();

        void setChangeVal(double cP, double cI, double cD);







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

    double brakeP = 0;
    double brakeD = 0;
    double brakePrevL = 0;
    double brakePrevR = 0;
    double targetBrakeL = 0;
    double targetBrakeR = 0;

    void setPower(double pow);
    void setTurnPower(double pow);

    int curveDrive(int inputVal);

    void brake(double in1, double in2);
    

    double posX;
    double posY;
    pros::controller_digital_e_t brakeButton = pros::E_CONTROLLER_DIGITAL_Y;

    double kCurve = 0.1;
    double aCurve = 0.4;
    bool braking = false;
    double longTime = 5000;
    bool inertialIn = true;
    bool runMap = true;
    bool curBreak = false;
    double resetBrakes = true;

    double changeP = 0.01;
    double changeI = 0.01;
    double changeD = 0.01;

    

};



