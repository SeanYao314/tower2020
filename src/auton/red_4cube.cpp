#include "robot.hpp"
#include "okapi/api.hpp"

using namespace okapi;

void red_4cube() {
     //flipout sequence 
    chassis.moveDistance(3_in);
    arm_drive(1);
    intake_drive(200,200);
    pros::delay(200);
    intake_drive(0,0);
    pros::delay(200);
    intake_drive(200,200);
    pros::delay(200);
    arm_drive(0);
    chassis.moveDistance(-3_in);

    //intake cubes 
    intake_drive(-80,-80);
    pros::delay(400);
    intake_drive(0,0);
    pros::delay(400);
    intake_drive(160,160);
    chassis.setMaxVelocity(140);
    chassis.moveDistance(51.5_in);

    intake_drive(140, 140);

    // stack
    pros::delay(400);
    intake_drive(10,10);
    chassis.setMaxVelocity(100);
    chassis.moveDistance(-42.5_in);
    pros::delay(500);
    chassis.setMaxVelocity(60);
    chassis.turnAngle(-131.5_deg);
    intake_drive(-200,-200);
    pros::delay(160);
    intake_drive(0,0);

    chassis.moveDistance(13.5_in);
    lever_drive(200);
    pros::delay(3100);
    lever_drive(0);
    chassis.setMaxVelocity(170);
    chassis.moveDistance(4_in);
    intake_drive(-200,-200);
    chassis.setMaxVelocity(90);
    chassis.moveDistance(-29_in);
    intake_drive(0,0);

}