#include "robot.hpp"
#include "recording.h"
#include "screen.h"

using namespace okapi;
using namespace std;
using namespace recording;

void skill_auton();

// void gyroTurnL(double targetAngle, double speed) {
//     int threshold = 10;
//     int cPos = imu.get_rotation();
//     int tPos = targetAngle;
//     int aDelta = abs(cPos - tPos);
//     int delta = tPos - cPos;

//     std::cout << "the target position is " << tPos << endl;

//     while (aDelta > threshold) {
//         // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
//         // double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;
//         double speedCoef = ((10/9)*delta)/speed;
//         chassis->getModel()->tank(speedCoef*speed/10, -speedCoef*speed/10);
//         //chassis->getModel().tank(speed, -speed);

//         cPos = imu.get_rotation();
//         aDelta = abs(cPos - tPos);
//         delta = tPos - cPos;
//         //std::cout << "the current position is " << speedCoef*speed << endl;
//     }
//     chassis->getModel()->tank(0,0);
// }

// void gyroRTurnL(double degree, double speed) {
//     int cPos = imu.get_rotation();
    
//     gyroTurnL(degree+(cPos/10), speed);
// }

// void gyroTurnR(double targetAngle, double speed) {
//     int threshold = 100;
//     int cPos = abs((int)imu.get_rotation() - 360) % 360;
//     int tPos = targetAngle;
//     int aDelta = abs(cPos - tPos);
//     int delta = tPos - cPos;

//     std::cout << "the target position is " << tPos << endl;

//     while (aDelta > threshold) {
//         // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
//         //rational expression ^^
//         // double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;
//         // cuberoot ^^
//          double speedCoef = ((10/9)*delta)/speed;
//          //linear ^^
//         chassis->getModel()->tank(-speedCoef*speed/10, speedCoef*speed/10);
//         //chassis->getModel().tank(speed, -speed);

//         cPos = imu.get_rotation();
//         aDelta = abs(cPos - tPos);
//         delta = tPos - cPos;
//         //std::cout << "the current position is " << speedCoef*speed << endl;
//     }
//     chassis->getModel()->tank(0,0);
// }
// void gyroRTurnR(double degree, double speed) {
//     int cPos = imu.get_rotation();
    
//     gyroTurnR(degree+(cPos/10), speed);
// }

void turningGyro(int degrees) {
    const double pi = 3.14159265358979323846;
    const double chassis_power = 0.5;
    double current_pos = gyro.get_value() / 10.0;
    double target_pos = current_pos + degrees;

    double delta = sin((target_pos - current_pos) / 180 * pi);
    double last_delta = abs(delta);
    double current_delta = last_delta;

    while (current_delta <= last_delta) {
        last_delta = current_delta;
        if (delta < 0) {
            chassis->getModel()->tank(-chassis_power, chassis_power);
        } else {
            chassis->getModel()->tank(chassis_power, -chassis_power);
        }
        current_pos = gyro.get_value() / 10;
        current_delta = abs(sin((target_pos - current_pos) / 180 * pi));

        cout << "target pos = " << target_pos << ", current pos = " << current_pos << ", delta = " << delta << endl;
        pros::delay(20);
    }
    chassis->getModel()->stop();
}

void imuNoSleuth(double degrees, double speed) {
    double cPos;
    double degCoef = (17/18) * degrees;
    if(speed > 0) {
        cPos = imu.get_rotation();
    } else if(speed < 0) {
        cPos = abs(int(imu.get_rotation() - 360)) % 360;
    }
    double tPos = degrees;
    double aDelta = tPos - cPos;
    int threshold = 1;
    double speedCoef = 1;

    while(aDelta > threshold) {
        if(35 > aDelta > 53) {
            speedCoef = 1/2;
        } else if(0 > aDelta > 34) {
            speedCoef = 1/3;
        } else if(aDelta < 0) {
            speedCoef = -1/2;
        } else {
            speedCoef = 1;
        } 
        chassis->getModel()->tank(-speedCoef * speed/200, speedCoef * speed/200);
        if(speed > 0) {
            cPos = imu.get_rotation();
        } else if(speed < 0) {
            cPos = abs(int(imu.get_rotation() - 360)) % 360;
       }
        tPos = degrees;
        aDelta = abs(tPos - cPos);
    }

    if(aDelta > threshold) {
        // imuNoSleuth(degCoef,-speed);
        chassis->moveDistance(10_in);
    } else {
        chassis->getModel()->tank(0,0);
    }
    
}
void red_unprotect() {
    arm_drive(1);
    pros::delay(350);
    arm_drive(0);

    intake_drive(200,200);
    chassis->getModel()->tank(200,200);
    pros::delay(180);
    chassis->getModel()->tank(0,0);

    pros::delay(700);
    intake_drive(0,0);

    arm_drive(4);
    pros::delay(500);
    arm_drive(0);


    chassis->setMaxVelocity(86);
    intake_drive(-200,-200);
    chassis->moveDistance(43_in); //smaller because 6096A anti tip pushes out robot
    intake_drive(-25,-25);

    // imuNoSleuth(175,90);
    // intake_drive(200, 200);
    // pros::delay(70);
    // intake_drive(0,0);
    // chassis->moveDistance(48.63_in);

    chassis->moveDistance(-28_in);
    pros::delay(100);
    intake_drive(200,200);
    pros::delay(70);
    intake_drive(0,0);
    imuNoSleuth(110, 70);

    chassis->setMaxVelocity(200);
    chassis->moveDistance(10_in);
    pros::delay(200);
    intake_drive(20,20);
    

    lever_drive(-200);
    pros::delay(1800);
    lever_drive(0);

    intake_drive(0,0);

    lever_drive(-50);
    pros::delay(700);
    lever_drive(0);


    chassis->getModel()->tank(200,200);
    pros::delay(75);
    chassis->getModel()->tank(0,0);
    intake_drive(200,200);
    chassis->moveDistance(-10_in);
    intake_drive(0,0);

}
void blue_unprotect() {
    chassis->setMaxVelocity(100);
    intake_drive(200,200);
    pros::delay(500);
    intake_drive(0,0);

    arm_drive(1);
    pros::delay(410);
    arm_drive(0);

    pros::delay(500);

    intake_drive(-140,-140);
    chassis->moveDistance(46.5_in); //smaller because 6096A anti tip pushes out robot
    intake_drive(-25,-25);

    // chassis->moveDistance(-33_in);
    // pros::delay(100);
    // intake_drive(200,200);
    // pros::delay(70);
    // intake_drive(0,0);
    // imuNoSleuth(-105, 70);
    // chassis->moveDistance(8_in);
    // pros::delay(200);
    // intake_drive(20,20);
    
    turningGyro(-160);
    intake_drive(200, 200);
    pros::delay(70);
    intake_drive(0,0);
    chassis->moveDistance(43.63_in);

    lever_drive(-200);
    pros::delay(1800);
    lever_drive(0);

    intake_drive(0,0);

    lever_drive(-50);
    pros::delay(700);
    lever_drive(0);

    chassis->setMaxVelocity(200);
    intake_drive(200,200);
    chassis->moveDistance(-10_in);
    intake_drive(0,0);
}

void red_protect() {

}

void blue_protect() {

}

void autonomous() {

    auto program = screen::get_selected_program();
    if (program == "Joker") {
        skill_auton();
        return;
    } else if (program == "Alpha") {
        blue_unprotect();
        return;
    } else if (program == "Beta") {
        blue_protect();
    } else if (program == "Kilo") {
        red_unprotect();
    } else if (program == "Lima") {
        red_protect();
    } else {
        red_unprotect();

    }

}
