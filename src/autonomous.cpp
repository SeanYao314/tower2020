#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace std;

void gyroTurn(double targetAngle, double speed) {
    int threshold = 50;
    int cPos = gyro.get_value();
    int tPos = targetAngle * 10;
    int aDelta = abs(cPos - tPos);
    int delta = tPos - cPos;

    cout << "the target position is " << tPos << endl;

    while (aDelta > threshold) {
        // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
        double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;
        chassis.tank(speedCoef*speed/20, -speedCoef*speed/20);
        //chassis.tank(speed, -speed);

        cPos = gyro.get_value();
        aDelta = abs(cPos - tPos);
        delta = tPos - cPos;
        //std::cout << "the current position is " << speedCoef*speed << endl;
    }
    chassis.tank(0,0);
}

void gyroRTurn(double degree, double speed) {
    int cPos = gyro.get_value();
    
    gyroTurn(degree+(cPos/10), speed);
}


void autonomous() {
    std::cout << "turn: " << gyro.get_value();
    pros::delay(500);
    // recording::replay();
    // gyroTurn(135,170);
    gyroRTurn(-90, -170);


}

