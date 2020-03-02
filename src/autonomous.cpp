#include "robot.hpp"
#include "recording.h"
#include "screen.h"

using namespace okapi;
using namespace std;
using namespace recording;

void skill_auton();


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


void gyroTurning(int degrees, double chassis_power) {
    const double pi = 3.14159265358979323846;
    double current_pos = gyro.get_value() / 10.0;
    double target_pos = current_pos + degrees;

    double delta = sin((target_pos - current_pos) / 180 * pi);
    double last_delta = abs(delta);
    double current_delta = last_delta;

    while (current_delta <= last_delta) {
        last_delta = current_delta;
        if (delta < 0) {
            chassis->getModel()->tank(chassis_power, -chassis_power);
        } else {
            chassis->getModel()->tank(-chassis_power, chassis_power);
        }
        current_pos = gyro.get_value() / 10;
        current_delta = abs(sin((target_pos - current_pos) / 180 * pi));

        cout << "target pos = " << target_pos << ", current pos = " << current_pos << ", delta = " << delta << endl;
        pros::delay(20);
    }
    chassis->getModel()->stop();
}

void moveForwardPower(int cycles, double power) {
    for (int i=0; i<cycles; ++i) {
        chassis->getModel()->tank(power, power);
        pros::delay(50);
    }
    chassis->stop();
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
    double aDelta = abs(tPos - cPos);
    int threshold = 2.5;
    double speedCoef = 1;

    while(aDelta > threshold) {
        if(35 > aDelta > 53) {
            speedCoef = 1/2;
        } else if(0 > aDelta > 34) {
            speedCoef = 1/7;
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
}

/* ---------------------------------- START OF AUTONS ---------------------------------- */
/* ------------RED 5 CUBES--------------- */
void red_unprotect() {
    chassis->getModel()->tank(200,200);
    pros::delay(100);
    chassis->getModel()->tank(0,0);
    arm_drive(4);
    pros::delay(90);
    intake_drive(200,200);
    pros::delay(130);
    arm_drive(0);

    intake_drive(0,0);

    chassis->setMaxVelocity(66);
    intake_drive(-190,-190);
    chassis->moveDistance(36_in); //smaller because 6096A anti tip pushes out robot
    intake_drive(-50,-50);

    // imuNoSleuth(175,90);
    // intake_drive(200, 200);
    // pros::delay(70);
    // intake_drive(0,0);
    // chassis->moveDistance(48.63_in);

    chassis->moveDistance(-23_in);
    pros::delay(200);

    intake_drive(200,200);
    pros::delay(70);
    intake_drive(0,0);

    chassis->setMaxVelocity(2000);
    chassis->turnAngle(-189_deg);

    chassis->setMaxVelocity(200);
    chassis->getModel()->tank(100,200);
    pros::delay(450);
    chassis->getModel()->tank(0,0);
    intake_drive(40,40);
    
    chassis->getModel()->tank(0.07,0.21);

    lever_drive(-200);
    pros::delay(1700);
    lever_drive(0);

    chassis->getModel()->tank(0,0);

    intake_drive(0,0);

    lever_drive(-50);
    pros::delay(900);
    lever_drive(0);


    chassis->getModel()->tank(100,100);
    pros::delay(90);
    chassis->getModel()->tank(0,0);
    intake_drive(200,200);
    pros::delay(100);
    chassis->getModel()->tank(-140,-140);
    pros::delay(400);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);

    lever_drive(200);
    pros::delay(2000);
    lever_drive(0);
}
/* ------------BLUE 5 CUBE--------------- */
void blue_unprotect() {
    chassis->getModel()->tank(200,200);
    pros::delay(100);
    chassis->getModel()->tank(0,0);
    arm_drive(4);
    pros::delay(90);
    intake_drive(200,200);
    pros::delay(130);
    arm_drive(0);

    intake_drive(0,0);

    chassis->setMaxVelocity(66);
    intake_drive(-190,-190);
    chassis->moveDistance(36_in); //smaller because 6096A anti tip pushes out robot
    intake_drive(-50,-50);

    // imuNoSleuth(175,90);
    // intake_drive(200, 200);
    // pros::delay(70);
    // intake_drive(0,0);
    // chassis->moveDistance(48.63_in);

    chassis->moveDistance(-23_in);
    pros::delay(200);

    intake_drive(200,200);
    pros::delay(70);
    intake_drive(0,0);

    chassis->setMaxVelocity(2000);
    chassis->turnAngle(200_deg);

    chassis->setMaxVelocity(200);
    chassis->getModel()->tank(100,200);
    pros::delay(450);
    chassis->getModel()->tank(0,0);
    intake_drive(40,40);
    
    chassis->getModel()->tank(0.07,0.04);

    lever_drive(-200);
    pros::delay(1400);
    lever_drive(0);

    chassis->getModel()->tank(0,0);

    intake_drive(0,0);

    lever_drive(-50);
    pros::delay(900);
    lever_drive(0);


    chassis->getModel()->tank(200,200);
    pros::delay(160);
    chassis->getModel()->tank(0,0);
    intake_drive(200,200);
    pros::delay(100);
    chassis->getModel()->tank(-140,-140);
    pros::delay(400);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);

    lever_drive(200);
    pros::delay(2000);
    lever_drive(0);
}
/* ------------RED 3 CUBE-------------- */
void red_protect() {
    intake_drive(-130,-130);
    chassis->getModel()->tank(0.34,0.34);
    arm_drive(4);
    pros::delay(340);
    arm_drive(0);
    pros::delay(1280);
    chassis->getModel()->tank(0,0);

    //turn
    pros::delay(400);
    chassis->setMaxVelocity(1900);
    chassis->turnAngle(131_deg);

    chassis->setMaxVelocity(70);

    //intake
    intake_drive(-80,-80);
    chassis->moveDistance(23_in);
    intake_drive(-30,-30);
    chassis->moveDistance(-11_in);
  
    //turn
    chassis->setMaxVelocity(1600);
    chassis->turnAngle(61_deg);

    intake_drive(200,200);
    pros::delay(120);
    intake_drive(0,0);

    //stack & release
    chassis->setMaxVelocity(200);
    chassis->getModel()->tank(0.3,0.6);
    pros::delay(1270);
    chassis->getModel()->tank(0,0);
    intake_drive(30,30);
    chassis->getModel()->tank(0.04,0.12);
    lever_drive(-200);
    pros::delay(1700);
    lever_drive(0);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);
    lever_drive(-50);
    pros::delay(900);
    lever_drive(0);
    chassis->getModel()->tank(100,100);
    pros::delay(90);
    chassis->getModel()->tank(0,0);
    intake_drive(200,200);
    pros::delay(100);
    chassis->getModel()->tank(-140,-140);
    pros::delay(400);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);
    lever_drive(200);
    pros::delay(1700);
    lever_drive(0);
}
/* ------------BLUE 3 CUBE--------------- */
void blue_protect() {
    intake_drive(-100,-100);
    chassis->getModel()->tank(0.32,0.32);
    arm_drive(4);
    pros::delay(270);
    arm_drive(0);
    pros::delay(1200);
    chassis->getModel()->tank(0,0);

    //turn
    pros::delay(400);
    chassis->setMaxVelocity(1900);
    chassis->turnAngle(-126_deg);
  
    chassis->setMaxVelocity(70);

    //intake
    intake_drive(-70,-70);
    chassis->moveDistance(23_in);

    intake_drive(-30,-30);
    chassis->moveDistance(-9_in);
    //turn
    chassis->setMaxVelocity(1800);
    chassis->turnAngle(-53_deg);

    intake_drive(200,200);
    pros::delay(120);
    intake_drive(0,0);

    //stack & release
    chassis->setMaxVelocity(200);
    chassis->getModel()->tank(0.5,0.3);
    pros::delay(1060);
    chassis->getModel()->tank(0,0);
    intake_drive(30,30);
    chassis->getModel()->tank(0.21,0.7);
    lever_drive(-200);
    pros::delay(1700);
    lever_drive(0);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);
    lever_drive(-50);
    pros::delay(900);
    lever_drive(0);
    chassis->getModel()->tank(100,100);
    pros::delay(90);
    chassis->getModel()->tank(0,0);
    intake_drive(200,200);
    pros::delay(100);
    chassis->getModel()->tank(-140,-140);
    pros::delay(200);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);
    lever_drive(200);
    pros::delay(2000);
    lever_drive(0);
}

/* -------------------------------7 CUBE RED----------------------------------- */
void red_seven_cube() {
    // chassis->getModel()->tank(0.3,0.3);
    // pros::delay(500);
    // chassis->getModel()->tank(0,0);
    // arm_drive(4);
    // pros::delay(140);
    // intake_drive(200,200);
    // pros::delay(210);
    // arm_drive(0);

    // intake_drive(0,0);

    // chassis->setMaxVelocity(76);
    // intake_drive(-200,-200);
    // chassis->moveDistance(26_in); 

    // pros::delay(20000);

    gyroTurning(-14,0.3);
    intake_drive(-120,-120);
    chassis->getModel()->tank(0.3,0.55);
    pros::delay(1100);
    chassis->getModel()->tank(0,0);

    // intake_drive(-120,-120);
    // chassis->moveDistance(17.9_in);

    // pros::delay(350);
    // intake_drive(-15,-15);

    // pros::delay(130);
    // moveForwardPower(6,-0.6);
    // moveForwardPower(7, -0.3);

    // pros::delay(300);
    // gyroTurning(-18, 0.3);

    // intake_drive(-130,-130);
    // moveForwardPower(12,0.5);
    // pros::delay(470);
    // moveForwardPower(12,-0.5);

    // pros::delay(850);
    // gyroTurning(18,0.3);

    // intake_drive(-20,-20);
    // chassis->setMaxVelocity(86);


    // chassis->setMaxVelocity(76);
    // chassis->moveDistance(-26_in);
    // pros::delay(200);

    // intake_drive(200,200);
    // pros::delay(70);     
    // intake_drive(0,0);

    // // chassis->setMaxVelocity(2000);
    // // chassis->turnAngle(-189_deg);

    // // chassis->setMaxVelocity(200);
    // // chassis->getModel()->tank(100,200);
    // // pros::delay(450);
    // // chassis->getModel()->tank(0,0);
    // // intake_drive(40,40);
    
    // // chassis->getModel()->tank(0.07,0.21);

    // // lever_drive(-200);
    // // pros::delay(1700);
    // // lever_drive(0);

    // // chassis->getModel()->tank(0,0);

    // // intake_drive(0,0);

    // // lever_drive(-50);
    // // pros::delay(900);
    // // lever_drive(0);


    // // chassis->getModel()->tank(100,100);
    // // pros::delay(90);
    // // chassis->getModel()->tank(0,0);
    // // intake_drive(200,200);
    // // pros::delay(100);
    // // chassis->getModel()->tank(-140,-140);
    // // pros::delay(400);
    // // chassis->getModel()->tank(0,0);
    // // intake_drive(0,0);

    // // lever_drive(200);
    // // pros::delay(2000);
    // // lever_drive(0);

}
/* -------FOUR CUBE UNPROTECTED----- */
void four_cube_red() {
    intake_drive(-100,-100);
    pros::delay(100);
    chassis->getModel()->tank(0.35,0.35);
    arm_drive(4);
    pros::delay(390);
    arm_drive(0);
    pros::delay(1200);
    chassis->getModel()->tank(0,0);

    //turn
    pros::delay(400);
    chassis->setMaxVelocity(1900);
    chassis->turnAngle(-120_deg);
    pros::delay(400);
  
    chassis->setMaxVelocity(95);

    //intake
    intake_drive(-90,-90);
    chassis->moveDistance(16_in);

    chassis->getModel()->tank(0.1,0.45);
    pros::delay(550);
    chassis->getModel()->tank(0.45,0.1);
    pros::delay(550);
    chassis->getModel()->tank(0.39,0.39);
    pros::delay(740);
    chassis->getModel()->tank(0,0);

    pros::delay(100);
    intake_drive(-20,-20);

    chassis->getModel()->tank(-0.7,-0.4);
    pros::delay(1290);
    chassis->getModel()->tank(-0.25,0.9);
    pros::delay(450);
    chassis->getModel()->tank(0.15,0.15);
    pros::delay(500);
    chassis->getModel()->tank(0,0);

    chassis->setMaxVelocity(1800);
    chassis->turnAngle(-27_deg);
    chassis->setMaxVelocity(80);
    chassis->getModel()->tank(0.25,0.33);

    intake_drive(200,200);
    pros::delay(230);
    intake_drive(0,0);

    chassis->getModel()->tank(0.34,0.44);
    pros::delay(800);
    chassis->getModel()->tank(0.2,0.235);

    chassis->getModel()->tank(1,1);
    pros::delay(200);
    chassis->getModel()->tank(0,0);

    intake_drive(30,30);

    lever_drive(-200);
    pros::delay(1700);
    lever_drive(0);

    chassis->getModel()->tank(0,0);

    intake_drive(0,0);

    lever_drive(-50);
    pros::delay(900);
    lever_drive(0);


    chassis->getModel()->tank(100,100);
    pros::delay(60);
    chassis->getModel()->tank(0,0);
    intake_drive(200,200);
    pros::delay(100);
    chassis->getModel()->tank(-140,-140);
    pros::delay(540);
    chassis->getModel()->tank(0,0);
    intake_drive(0,0);

    lever_drive(200);
    pros::delay(2000);
    lever_drive(0);


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
        red_seven_cube();
        cout << "yaw = " << imu.get_yaw() << ", cPos = " << imu.get_rotation() << ", k = " << sin(imu.get_rotation()) << endl;
    }


}
