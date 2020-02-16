#include "api.h"
#include "robot.hpp"
#include "okapi/api.hpp"
#include "screen.h"
#include "storage.h"

using namespace std;
using namespace pros;
using namespace okapi;

okapi::Motor intake_motor_left(INTAKE_MOTOR_LEFT);
okapi::Motor intake_motor_right(INTAKE_MOTOR_RIGHT);
okapi::Motor lever_motor(LEVER_MOTOR);
okapi::Motor arm_motor(ARM_MOTOR);
pros::Controller master (E_CONTROLLER_MASTER);

pros::Imu imu(1);
pros::ADIGyro gyro (GYRO_PORT);

/* choose only one of the following: either separated chassis motors, or the chassis controller */
//chassis motors
// okapi::Motor chassis_right_rear(CHASSIS_RIGHT_REAR);
// okapi::Motor chassis_right_front(CHASIIS_RIGHT_FRONT);
// okapi::Motor chassis_left_rear(CHASSIS_LEFT_REAR);
// okapi::Motor chassis_left_front(CHASSIS_LEFT_FRONT);

const auto WHEEL_DIAMETER = 4.5_in;
const auto CHASSIS_WIDTH = 9.3_in;

// chassis controller
// okapi::ChassisControllerIntegrated chassis = ChassisControllerFactory::create(
//   {-CHASSIS_LEFT_FRONT, -CHASSIS_LEFT_REAR}, 
//   {CHASIIS_RIGHT_FRONT, CHASSIS_RIGHT_REAR},
//   AbstractMotor::gearset::green,
//   {WHEEL_DIAMETER, CHASSIS_WIDTH}
// );

// auto chassis = ChassisControllerBuilder(
//   {-CHASSIS_LEFT_FRONT, -CHASSIS_LEFT_REAR}, 
//   {CHASIIS_RIGHT_FRONT, CHASSIS_RIGHT_REAR},

//     IterativePosPIDController::Gains{0.00420, 0.0000, 0.00000}, //distance
//     IterativePosPIDController::Gains{0.0024, 0.0005, 0.00002},  //angle
//     IterativePosPIDController::Gains{0.003, 0.0065, 0.000045},  //turn

//   AbstractMotor::gearset::green,
//   {WHEEL_DIAMETER, CHASSIS_WIDTH}
// );

std::shared_ptr<okapi::OdomChassisController> chassis = ChassisControllerBuilder()
		.withMotors( {-CHASSIS_LEFT_FRONT, -CHASSIS_LEFT_REAR}, {CHASIIS_RIGHT_FRONT, CHASSIS_RIGHT_REAR})
		.withGains(
			{0.001, 0.0004, 0.00001}, // Distance controller gains
			{0.0001, 0.0001, 0.000}, // Turn controller gains
			{0.002, 0.01, 0.000001} // Angle controller gains (helps drive straight)
		)
		// green gearset, 4 inch wheel diameter, 8.125 inch wheelbase
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 8.125_in}, imev5GreenTPR})
		// specify the tracking wheels diameter (2.75 in), track (4 in), and TPR (360)
		.withOdometry({{2.75_in, 4.6_in}, quadEncoderTPR})
		.buildOdometry(); // build an odometry chassis


vector<AbstractMotor*>& get_motor_group() {
	static vector<AbstractMotor*> motor_group;

	if (motor_group.size() == 0) {
		auto left_motor = ((SkidSteerModel *)chassis->getModel().get())->getLeftSideMotor();
		auto right_motor = ((SkidSteerModel *)chassis->getModel().get())->getRightSideMotor();

		motor_group.push_back(left_motor.get());
		motor_group.push_back(right_motor.get());

		// motor_group.push_back(&chassis_left_front);
		// motor_group.push_back(&chassis_left_rear);
		// motor_group.push_back(&chassis_right_front);
		// motor_group.push_back(&chassis_right_rear);

		motor_group.push_back(&intake_motor_left);
		motor_group.push_back(&intake_motor_right);
		motor_group.push_back(&lever_motor);
		motor_group.push_back(&arm_motor);
	}

	return motor_group;
}

void notify_controller(const char * rumble_msg, const char * msg) {
	master.rumble(rumble_msg);
	master.print(0, 0, "%s", msg);
}

vector<recording::RecordUnit>& load_replaying() {
	static vector<recording::RecordUnit> empty;
	if (storage::is_slot_taken(screen::get_selected_program())) {
		return storage::get_program(screen::get_selected_program());
	} else {
		return empty; 
	}
}

void initialize()
{
	screen::setup_screen();
	screen::set_notif_handler(notify_controller);
	storage::load_all_programs();
	recording::set_motor_group(get_motor_group());
	recording::set_replay_loader(load_replaying);
}
