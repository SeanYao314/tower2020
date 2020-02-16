#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace std;



void gyroTurnL(double targetAngle, double speed) {
    int threshold = 10;
    int cPos = imu.get_rotation();
    int tPos = targetAngle;
    int aDelta = abs(cPos - tPos);
    int delta = tPos - cPos;

    cout << "the target position is " << tPos << endl;

    while (aDelta > threshold) {
        // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
        // double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;
        double speedCoef = ((10/9)*delta)/speed;
        chassis->getModel()->tank(speedCoef*speed/10, -speedCoef*speed/10);
        //chassis->getModel().tank(speed, -speed);

        cPos = imu.get_rotation();
        aDelta = abs(cPos - tPos);
        delta = tPos - cPos;
        //std::cout << "the current position is " << speedCoef*speed << endl;
    }
    chassis->getModel()->tank(0,0);
}

void gyroRTurnL(double degree, double speed) {
    int cPos = imu.get_rotation();
    
    gyroTurnL(degree+(cPos/10), speed);
}

void gyroTurnR(double targetAngle, double speed) {
    int threshold = 100;
    int cPos = abs((int)imu.get_rotation() - 360) % 360;
    int tPos = targetAngle;
    int aDelta = abs(cPos - tPos);
    int delta = tPos - cPos;

    cout << "the target position is " << tPos << endl;

    while (aDelta > threshold) {
        // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
        //rational expression ^^
        // double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;
        // cuberoot ^^
         double speedCoef = ((10/9)*delta)/speed;
         //linear ^^
        chassis->getModel()->tank(-speedCoef*speed/10, speedCoef*speed/10);
        //chassis->getModel().tank(speed, -speed);

        cPos = imu.get_rotation();
        aDelta = abs(cPos - tPos);
        delta = tPos - cPos;
        //std::cout << "the current position is " << speedCoef*speed << endl;
    }
    chassis->getModel()->tank(0,0);
}
void gyroRTurnR(double degree, double speed) {
    int cPos = imu.get_rotation();
    
    gyroTurnR(degree+(cPos/10), speed);
}

void imuNoSleuth(double degrees, double speed) {
    double cPos;
    if(speed > 0) {
        cPos = imu.get_rotation;
    } else if(speed < 0) {
        cPos = abs((int)imu.get_rotation() - 360) % 360;
    }
    double tPos = degrees;
    double aDelta = abs(tPos - cPos);
    int threshold = 3;
    double speedCoef = 1;

    while(aDelta > threshold) {
        if(1 > aDelta > 35) {
            speedCoef = 1/2;
        }
        chassis->getModel()->tank(speedCoef * speed, -speedCoef * speed);

    }
    pros::delay(200);
    if(aDelta > threshold) {
        imuNoSleuth(degrees,-speed);
    } else {
        chassis->getModel()->tank(0,0);
    }
    
}
void autonomous() {
    std::cout << "turn: " << imu.get_rotation();
    pros::delay(500);
    // recording::replay();
    imuNoSleuth(90,200);
    // gyroRTurn(270, -170);


}

