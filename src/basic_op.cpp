#include "main.h"
#include "okapi/api.hpp"
#include "robot.hpp"

using namespace okapi;
using namespace pros;
//chassis
void chassis_tank_drive(int left, int right) {
    const double chassis_movement_threshold = 0.05;
    chassis->getModel()->tank(right / 127.0, left / 127.0, chassis_movement_threshold);
}
//intake
void intake_drive(float left_intake_speed, float right_intake_speed) {
	intake_motor_left.moveVelocity(left_intake_speed);
	intake_motor_right.moveVelocity(-right_intake_speed);
}
//lever
void lever_drive(float lever_speed) {
	lever_motor.moveVelocity(lever_speed);
}

int armTarget = 0;
int armIterate = 0;

void arm_drive(int presetPos) {
	armIterate = presetPos;
	if (armIterate >= ARM_PRESETS_LEN) {
		armIterate = ARM_PRESETS_LEN;
	}
	armTarget = ARM_PRESETS[armIterate];
	// master.print(0, 0, "Don't read this sentence %d", armTarget);
	arm_motor.moveAbsolute(armTarget, 200);
}