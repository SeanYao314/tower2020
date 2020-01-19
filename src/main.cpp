#include "main.h"
#include "okapi/api.hpp"
#include "robot.hpp"

using namespace okapi;
using namespace pros;

void disabled() {}

void competition_initialize() {

}

void opcontrol() {
	while (true) {
		//chassis stuff
		chassis_control();

		//intake
		intake_control();

		//lever stuff
		lever_control(1);

		// lift from korvex
		arm_control();
		
		arm_control2();
		
		pros::delay(20);
	}
}
