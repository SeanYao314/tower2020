#include "robot.hpp"
#include "okapi/api.hpp"

using namespace okapi;

void skill1() {
    chassis.setMaxVelocity(120);
    chassis.moveDistance(48_in);
    pros::delay(1000);
    chassis.moveDistance(-48_in);
}

void skill2() {

}

void skill3() {
    
}