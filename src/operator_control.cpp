#include "main.h"
#include "okapi/api.hpp"
#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace pros;


void gyroTest() {
	if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X) > 0) {
		std::cout << "Distance: " << imu.get_rotation();
	} 
}

//where the arm is
int armPos = 0;

//presets
void arm_control() {
	int armTarget = 0;
	int armIterate = 0;

	if (master.get_digital_new_press(DIGITAL_RIGHT)) {
		//descore high
		arm_drive(3);
		armPos = 3;
	} else if (master.get_digital_new_press(DIGITAL_UP)) {
		//score high
		arm_drive(4);
		armPos = 4;
	} else if (master.get_digital_new_press(DIGITAL_LEFT)) {
		//descore low
		arm_drive(1);
		armPos = 1;
	} else if(master.get_digital_new_press(DIGITAL_DOWN)) {
		//score low
		arm_drive(2);
		armPos = 2;
	} else if (master.get_digital_new_press(DIGITAL_Y) == 1 && !armPos == 0) {
		//original pos
		armIterate = 0;
		armPos = 0;
		arm_motor.setBrakeMode(AbstractMotor::brakeMode::coast);
		arm_motor.moveVoltage(12000);
		pros::delay(500);
		arm_motor.moveVoltage(0);
	}
}

//manual drive for arm
void arm_control2() {

	if(master.get_digital(E_CONTROLLER_DIGITAL_B)) {

		arm_motor.moveVoltage(-12000);
		arm_motor.setBrakeMode(AbstractMotor::brakeMode::hold);
		// pros::delay(50);
		// arm_motor.move_voltage(0);
	} else if (master.get_digital(E_CONTROLLER_DIGITAL_A)) {

		arm_motor.moveVoltage(5000);
		arm_motor.setBrakeMode(AbstractMotor::brakeMode::hold);
		// pros::delay(50);
		// arm_motor.move_voltage(0);

	} else if (master.get_digital(E_CONTROLLER_DIGITAL_X)) {

		arm_motor.setBrakeMode(AbstractMotor::brakeMode::coast);
		arm_motor.moveVoltage(12000);
		pros::delay(300);
		arm_motor.moveVoltage(0);

	}  else if (armPos == 0) {
		//so arm control 1&2 dont collide
		arm_motor.moveVoltage(0);
	}

}


//for chassis
void chassis_control() {
	int left_power = master.get_analog(ANALOG_LEFT_Y);
	int right_power = master.get_analog(ANALOG_RIGHT_Y);
	chassis_tank_drive(left_power, right_power);
}

void auton_simulator() {
	if (master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
		int x_power = master.get_analog(ANALOG_RIGHT_X);
		if (abs(x_power) > 120) {
			autonomous();
		}

		int left_x_power = master.get_analog(ANALOG_LEFT_X);
		if (abs(left_x_power) > 120) {
			recording::printout();
		}
	}
}

//intake
void intake_control() {
	if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
		intake_drive(-200, -200);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_L2) && armPos == 0) {
		intake_drive(190,190);
	} else if((master.get_digital(E_CONTROLLER_DIGITAL_L2) && armPos == 2) || (master.get_digital(E_CONTROLLER_DIGITAL_L2) && armPos == 4)) {
		//slower for scoring in towers
		pros::delay(15);
		intake_drive(96,96);
	}  else if((master.get_digital(E_CONTROLLER_DIGITAL_L2) && armPos == 1) || (master.get_digital(E_CONTROLLER_DIGITAL_L2) && armPos == 3)) {
		//slower for scoring in towers
		pros::delay(15);
		intake_drive(200,200);
	} else {
		intake_drive(0,0);
	}
}
int leverEndPos = -6550;
int leverStartPos= 138;
int tray_speed = 12.5;
int intakeSpeed = 34.0;

void lever_control() {
	if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
		intake_drive(intakeSpeed, intakeSpeed);
		arm_motor.setBrakeMode(AbstractMotor::brakeMode::hold);
		if(lever_motor.getPosition() < -3550) {
			tray_speed = 10;
			intakeSpeed = 0;
		} else if(lever_motor.getPosition() >= -3550) {
			tray_speed = 200;
			intakeSpeed = 34.0;
		} 
		lever_motor.moveAbsolute(leverEndPos, E_CONTROLLER_DIGITAL_R1*tray_speed);
		//intake pushes out lever slightly
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		arm_motor.setBrakeMode(AbstractMotor::brakeMode::coast);
		intake_drive(0,0);
		lever_motor.moveAbsolute(leverStartPos, E_CONTROLLER_DIGITAL_R1*200);
	} else {
		lever_drive(0);
	}
}
