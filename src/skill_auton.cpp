#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace std;
using namespace recording;

void replay_dp(vector<vector<int>>&);
void turning(int);

double exponential_speed_calculation(int distance_in_tick, double max_speed, double min_speed) {
    
    double exp_speed = pow(abs(distance_in_tick), 1 / 3.0) / 20.0;
    if (exp_speed > max_speed) {
        exp_speed = max_speed;
    } else if (exp_speed < min_speed) {
        exp_speed = min_speed;
    }
    if (distance_in_tick < 0) {
        exp_speed *= -1;
    }
    return exp_speed;
}

void move_straight(double distance_in_inch, double power) {
    const double pi = 3.14159265358979323846;
    const double inch_per_tick = 0.013962634015955;
    const double threshold = 0.5 / inch_per_tick;
    int target_position = distance_in_inch / inch_per_tick;

    auto left_motor = ((SkidSteerModel *)chassis->getModel().get())->getLeftSideMotor();
    auto right_motor = ((SkidSteerModel *)chassis->getModel().get())->getRightSideMotor();

    left_motor->tarePosition();
    right_motor->tarePosition();

    int old_delta = abs(target_position);
    int new_delta = old_delta - 1;
    double left_power = power;
    double right_power = power;
    double left_coef = 1.0;
    double right_coef = 1.0;

    double initial_position = gyro.get_value();

    while (new_delta <= old_delta) {
        old_delta = new_delta;

        left_power = exponential_speed_calculation(old_delta, power, 0.1);
        right_power = left_power;

        double current_position = gyro.get_value();
        double delta_degree = (current_position - initial_position) / 10;

        if (sin(delta_degree / 180 * pi) > sin(5 / 180 * pi)) {
            left_coef *= 0.999;
            right_coef *= 1.001;
            std::cout << ">>> drift right, left_coef = " << left_coef << ", right_coef = " << right_coef << endl;
        } else if (sin(delta_degree / 180 * pi) < sin(-5 / 180 * pi)) {
            left_coef *= 1.001;
            right_coef *= 0.999;
            cout << "<<< drift left, left_coef = " << left_coef << ", right_coef = " << right_coef << endl;
        } else {
            cout << "moving straight, left_coef = " << left_coef << ", right_coef = " << right_coef << endl;
        }
        left_power *= left_coef;
        right_power *= right_coef;

        chassis->getModel()->tank(left_power, right_power);
        pros::delay(20);

        int current_encoder_position = max(abs(left_motor->getPosition()), abs(right_motor->getPosition()));
        new_delta = abs(target_position - current_encoder_position);

        cout << "left power = " << left_power << ", right power = " << right_power << endl;
        // cout << "old delta = " << old_delta << ", new delta = " << new_delta << endl;
    }
    chassis->stop();
}

/*
void autonomous() {
    // recording::replay();
    move_straight(24, 0.5);
}
*/

vector<AbstractMotor*>& get_motor_group();

void raise_the_arm_and_release_anti_tip() {
    intake_drive(-100,-100);
    chassis->getModel()->tank(0.32,0.32);
    pros::delay(100);
    arm_drive(4);
    pros::delay(300);
    arm_drive(0);
    pros::delay(400);
    chassis->stop();
}

void leverBack() {

}

void move_forward_take_9_cubes() {
    vector<vector<int>> data_points = {
        {23,1000,1000,0,0,0,0},
        {24,1200,1200,0,0,0,0},
        {25,1500,1500,0,0,0,0},
        {26,1900,1900,0,0,0,0},
        {27,2400,2400,0,0,0,0},
        {28,2800,2800,0,0,0,0},
        {29,3200,3200,0,0,0,0},
        {30,3600,3600,0,0,0,0},
        {31,4000,4000,0,0,0,0},
        {32,4000,3500,-12000,12000,0,0},
        {53,4000,4000,-12000,12000,0,0},
        {86,4000,2500,-12000,12000,0,0},
        {112,2500,4000,-12000,12000,0,0},
        {128,3200,3200,-12000,12000,0,0},
        {224,0,0,-1200,1200,0,0},
        {225,0,0,-1200,1200,0,0}
        };
    replay_dp(data_points);
}

void move_forward(int cycles, double power) {
    for (int i=0; i<cycles; ++i) {
        chassis->getModel()->tank(power, power);
        pros::delay(50);
    }
    chassis->stop();
}

void stack_9_cubes() {
    vector<vector<int>> data_points = {
{0, 6, -6, -1029, 579, 6, 0},
{1, 0, 0, -7805, 6745, 0, 0},
{2, 0, 0, -10780, 11057, -6, -6},
{3, 0, 0, -10841, 11008, 6, 0},
{4, 0, -6, -11587, 11544, -6, -6},
{5, 0, 6, -10700, 11002, 0, 6},
{6, 0, 0, -11143, 10965, 6, 0},
{7, 6, 0, -2717, 1836, -6, 0},
{8, 0, 0, -388, 240, -3696, 0},
{9, 12, -6, 345, -55, -11821, 0},
{10, 0, -6, 1152, -1109, -11981, 0},
{11, -6, 0, 1207, -936, -11815, 6},
{12, -6, -6, 1220, -1287, -11753, 0},
{13, 0, -6, 992, -1029, -11661, 0},
{14, 0, 0, 986, -1275, -11636, 6},
{15, 0, 0, 986, -1250, -11618, -6},
{16, 0, -6, 1207, -1337, -11642, 0},
{17, 0, -6, 1343, -1595, -11642, 0},
{18, 0, -6, 1355, -2131, -11630, 6},
{19, 0, -6, 1306, -2710, -11642, -6},
{20, 0, 0, 1207, -2778, -11648, 6},
{21, 0, 0, 1300, -2735, -11673, -6},
{22, 12, 0, 1010, -2125, -11864, 0},
{23, 0, 0, 850, -1743, -11969, 0},
{24, 0, -6, 647, -1374, -11919, 0},
{25, 172, 0, 918, -1343, -11876, 6},
{26, 6, 0, 634, -1072, -11827, 0},
{27, 0, 0, 296, -357, -11759, 0},
{28, 6, 0, 1207, -407, -11753, 6},
{29, 0, 0, 1300, -1010, -11796, 0},
{30, 0, -6, 1226, -1337, -11809, 0},
{31, 0, 0, 1355, -1324, -11802, 0},
{32, 0, 0, 1343, -1583, -11778, 0},
{33, 0, 0, 1712, -1756, -11784, 0},
{34, 0, -6, 2390, -2094, -11778, 6},
{35, 0, 0, 2544, -2538, -11796, 0},
{36, 0, -6, 2741, -2624, -11815, 6},
{37, 0, 0, 2784, -2624, 5008, 6},
{38, 0, 0, 3296, -3129, -8722, 0},
{39, 0, 1318, 4004, -3357, -3530, 6},
{40, 0, 1398, 4041, -3160, -5211, 0},
{41, 961, 1829, 4158, -2901, -5248, 0},
{42, 1318, 2864, 4158, -3031, -5599, 0},
{43, 1324, 3314, 3739, -3813, -5870, 6},
{44, 1232, 3388, 3437, -3942, -6185, 0},
{45, 1103, 3265, 3523, -3450, -6080, 0},
{46, 1103, 3517, 3899, -3234, -5723, 0},
{47, 1096, 3480, 3905, -3523, -5686, -6},
{48, 1078, 3437, 3979, -5427, -5624, 0},
{49, 1078, 3228, 3000, -7176, -5630, 0},
{50, 1078, 2895, 2267, -6246, -5168, 6},
{51, 1226, 1799, 3000, -5901, -5150, 0},
{52, 1103, 955, 3246, -4682, -5636, -6},
{53, 1343, 499, 3228, -3271, -5655, 0},
{54, 1010, 856, 3234, -2914, -5753, 0},
{55, 1170, 1004, 2784, -3135, -4811, 0},
{56, 1170, 942, 3406, -3166, -4836, 0},
{57, 912, 912, 3967, -3825, -4503, 6},
{58, -240, 1725, 4164, -3973, -4250, -6},
{59, -18, 1706, 4903, -4121, -4398, 0},
{60, -92, 1608, 5316, -4515, -4466, 0},
{61, -240, 1146, 5291, -4712, -4589, -6},
{62, -55, 1010, 4916, -4478, -4620, 0},
{63, 0, 998, 4996, -4848, -4601, 6},
{64, 0, 1004, 4022, -4885, -4447, -6},
{65, -6, 1010, 3560, -4410, -4189, 0},
{66, -6, 1010, 3363, -4540, -4398, 6},
{67, 6, 1010, 2790, -4546, -4497, 0},
{68, 0, 1010, 2643, -4558, -4589, 6},
{69, 0, 1010, 2538, -4534, -4447, 0},
{70, 0, 1004, 2858, -4842, -4466, 0},
{71, 0, 1010, 3215, -4675, -4146, -6},
{72, 0, 1010, 3881, -4441, -4343, -6},
{73, 6, 1004, 3930, -4090, -4528, 0},
{74, 0, 1010, 4152, -4503, -4620, 0},
{75, -6, -308, 4349, -4312, -4558, 6},
{76, -80, -31, 4478, -3696, -4558, 6},
{77, -123, -25, 4109, -3394, -4484, -6},
{78, 0, -6, 3930, -2827, -4226, 0},
{79, 0, 0, 3985, -3123, -4367, 6},
{80, 0, 0, 3043, -3166, -4269, 0},
{81, 0, 0, 2914, -3425, -4256, 6},
{82, 0, -6, 3721, -4829, -2723, 0},
{83, -12, -111, 8439, -9874, -1811, -6},
{84, -1263, -142, 10897, -10379, -998, 0},
{85, -5877, -4109, 11131, -11230, -684, 0},
{86, -9301, -6449, 10521, -10342, -55, 0},
{87, -9147, -7872, 10478, -10238, 0, 0},
{88, -10620, -8069, 11045, -10922, -6, 0},
{89, -10749, -8162, 11143, -11205, 6, 6},
{90, -11100, -9018, 9677, -11494, 0, 0},
{91, -11316, -9135, 9258, -10275, 0, 0},
{92, -11482, -8926, 7731, -7909, 6, 0},
{93, -11648, -7238, 7041, -7306, 6, -6},
{94, -5458, -5760, 6308, -6628, 6, -6},
{95, -4805, -5033, 5458, -6191, 6, -6},
{96, -5969, -5322, 4811, -5341, 0, 6},
{97, -5384, -5445, 4121, -4848, 6, 0},
{98, -5415, -5421, 3554, -4337, 0, 0},
{99, -5088, -5199, 2926, -3641, 0, 0},
{100, -4558, -4614, 1848, -3006, 6, -6},
{101, -3985, -4035, -6, -2409, 6, 6},
{102, -3363, -3437, 6, -1823, 0, 0},
{103, -2901, -2883, -6, -1250, 0, 6},
{104, -2840, -2920, -6, -832, 6, 0},
{105, -2840, -2871, 0, -641, 6, 0},
{106, -2310, -2402, 0, -160, 6, 0},
{107, -1866, -1854, 0, -6, 6, 0},
{108, -1515, -1435, -6, -6, 6, 0},
{109, -1066, -1189, 0, 0, 6, 0},
{110, -788, -825, 6, -6, 0, 0},
{111, -727, -733, 0, 0, 6, 0},
{113, -462, -499, 0, 0, -6, 0},
{114, -203, -136, 0, -6, 6, -6},
{115, -6, 0, 0, 0, 0, 0},
{157, 12, 0, -6, 0, 0, 0},
        };
    replay_dp(data_points);
}

void turning(int degrees, double chassis_power) {
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

void replay_dp(vector<vector<int>>& data_points) {

    vector<RecordUnit> replay_recording;
    for (int i=0; i<data_points.size(); ++i) {
        auto dp = data_points[i];
        RecordUnit ru;
        ru.tick = dp[0];
        for (int j=1; j<dp.size(); ++j) {
            ru.units.push_back(dp[j]);
        }
        replay_recording.push_back(ru);
    }

    std::cout << "about to replay " << replay_recording.size() << " steps" << endl;
    int starting_tick = replay_recording.front().tick;
    int ending_tick = replay_recording.back().tick;

    vector<AbstractMotor*>& motor_group = get_motor_group();    

    int r_index = 0;
    for (int t = starting_tick; t < ending_tick; ++t) {
        RecordUnit& unit = replay_recording[r_index];
        int tick = unit.tick;
        if (tick == t) {
            for (int j=0; j<motor_group.size(); ++j) {
                motor_group[j]->moveVoltage(unit.units[j]);
            }
            r_index ++;
        }
        pros::delay(50);
    }
}

void turningImu(int degrees, double chassis_power) {
    const double pi = 3.14159265358979323846;
    double current_pos = (imu.get_rotation() + (gyro.get_value()/10))/2;
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
        current_pos = (imu.get_rotation() + (gyro.get_value()/10))/2;
        current_delta = abs(sin((target_pos - current_pos) / 180 * pi));

        cout << "target pos = " << target_pos << ", current pos = " << current_pos << ", delta = " << delta << endl;
        pros::delay(20);
    }
    chassis->getModel()->stop();
}
void ten_Cubes() {
    intake_drive(-160,-160);
    move_forward(26, 0.4);
    chassis->getModel()->tank(0.1,-0.1);
    pros::delay(100);
    move_forward(30,0.4);
    pros::delay(200);
    chassis->getModel()->tank(0.4,0.2);
    pros::delay(1000);
    chassis->getModel()->tank(0.2,0.4);
    pros::delay(1800);
    chassis->getModel()->tank(0.4,0.2);
    pros::delay(1440);
    chassis->getModel()->tank(0.2,0.4);
    pros::delay(476);
    move_forward(23,0.4);
    // chassis->getModel()->tank(0,0);
    // move_forward(80,0.3);
}

void imuAbs(double degrees, double speed) {
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
    chassis->getModel()->tank(0,0);
}
void imuAbsolute(double speed, double degrees) {
    double cPos;
    if(speed > 0) {
        cPos = imu.get_rotation();
    } else if(speed < 0) {
        cPos = abs(int(imu.get_rotation() - 360)) % 360;
    }
    double tPos = degrees;
    double aDelta = abs(tPos - cPos);
    int threshold = 2.5;
    int rotationDir = (abs(tPos - cPos))/(tPos-cPos);
    while(aDelta > threshold) {
        chassis->getModel()->tank(speed/200 * rotationDir, speed/200 * rotationDir);
        if(speed > 0) {
            cPos = imu.get_rotation();
        } else if(speed < 0) {
            cPos = abs(int(imu.get_rotation() - 360)) % 360;
        }
        tPos = degrees;
        aDelta = abs(tPos - cPos);
        int rotationDir = (abs(tPos - cPos))/(tPos-cPos);
    }
    chassis->getModel()->tank(0,0);
}
void gyroTurnR(double targetAngle, double speed) {
    int threshold = 2;
    int cPos = abs((int)imu.get_rotation() - 360) % 360;
    int tPos = targetAngle;
    int aDelta = abs(cPos - tPos);
    int delta = tPos - cPos;

    std::cout << "the current position is " << cPos << endl;

    while (aDelta >= threshold) {
        // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
        double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;

        chassis->getModel()->tank(-speedCoef*speed/10, speedCoef*speed/10);

        cPos = imu.get_rotation();
        aDelta = abs(cPos - tPos);
        delta = tPos - cPos;
        //std::cout << "the current position is " << speedCoef*speed << endl;
    }
    chassis->getModel()->tank(0,0);
}

void turnP(int targetTurn, int voltageMax=127, bool debugLog=false) {
	float kp = 1.6;
	float ki = 0.8;
	float kd = 0.45;

	// the untouchables
	int voltageCap = 0;
	float voltage = 0;
	float errorCurrent;
	float errorLast;
	int errorCurrentInt;
	int errorLastInt;
	int sameErrCycles = 0;
	int same0ErrCycles = 0;
	int p;
	float i;
	int d;
	int sign;
	float error;
	int startTime = pros::millis();

	while(autonomous) {
		error = targetTurn - imu.get_rotation();
		errorCurrent = abs(error);
		errorCurrentInt = errorCurrent;
		sign = targetTurn / abs(targetTurn); // -1 or 1

		p = (error * kp);
		if (abs(error) < 10) { // if we are in range for I to be desireable
			i = ((i + error) * ki);
		}
		else
			i = 0;
		d = (error - errorLast) * kd;
		
		voltage = p + i + d;

		if(abs(voltage) > voltageMax) voltage = voltageMax * sign;

		// set the motors to the intended speed
		chassis->getModel()->tank(-voltage/127, voltage/127);

		// timeout utility
		if (errorLastInt == errorCurrentInt) {
			if (errorLast <= 2 and errorCurrent <= 2) { // saying that error less than 2 counts as 0
				same0ErrCycles +=1;
			}
			sameErrCycles += 1;
		}
		else {
			sameErrCycles = 0;
			same0ErrCycles = 0;
		}

		// exit paramaters
		if (same0ErrCycles >= 5 or sameErrCycles >= 60) { // allowing for smol error or exit if we stay the same err for .6 second
			chassis->stop();
			std::cout << pros::millis() << "task complete with error " << errorCurrent << " in " << (pros::millis() - startTime) << "ms" << std::endl;
			return;
		}
		
		// debug
		std::cout << pros::millis() << "error " << errorCurrent << std::endl;
		std::cout << pros::millis() << "voltage " << voltage << std::endl;
        	std::cout << pros::millis() << "theta " << imu.get_rotation() << std::endl;

		// for csv output, graphing the function
		// if (debugLog) std::cout << pros::millis() << "," << error << "," << voltage << std::endl;

		// nothing goes after this
		errorLast = errorCurrent;
		errorLastInt = errorLast;
		pros::delay(10);
	}
}

void skill_auton() {
    // chassis->getModel()->tank(0.5,0.5);
    // pros::delay(350);
    // chassis->getModel()->tank(0,0);
    // arm_drive(4);
    // pros::delay(160);
    // intake_drive(200,200);
    // pros::delay(130);
    // arm_drive(0);

    // intake_drive(0,0);

    // chassis->setMaxVelocity(66);
    // intake_drive(-190,-190);
    // chassis->moveDistance(36_in);
    // intake_drive(-10,-10);

    // pros::delay(280);
    // // turning(24, 0.4);
    // // turningImu(24,0.4);
    // chassis->setMaxVelocity(1200);
    // chassis->turnAngle(-42.5_deg);

    // move_forward(22,-0.7);
    // pros::delay(400);
    // imuAbsolute(0, -70);

    // pros::delay(200);
    // move_forward(10,-0.3);
    // pros::delay(200);
    // intake_drive(-190, -190);
    // move_forward(30,0.3);
    // pros::delay(100);

    // chassis->setMaxVelocity(54);
    // chassis->moveDistance(15_in);
    // intake_drive(-10,-10);
    /*
    chassis->stop();
    pros::delay(50);

    turning(-35, 0.25);
    pros::delay(100);

    move_forward(25, 0.5);
    pros::delay(50);
    intake_drive(-200,-200);
    pros::delay(900);
    intake_drive(0,0);
    pros::delay(500);

    stack_9_cubes();
    
    turning(90, 0.3);
    pros::delay(200);
    turning(15, 0.3);
    pros::delay(200);

    move_forward(15.5,-0.3);
    pros::delay(300);
    
    // first stack 
    lever_drive(200);
    intake_drive(-120,-120);
    chassis->getModel()->tank(0.4,0.2);
    pros::delay(1000);
    chassis->getModel()->tank(0.2,0.4);
    lever_drive(0);
    pros::delay(1000);
    move_forward(25,0.4);
    pros::delay(100);

    intake_drive(200,200);
    pros::delay(100);
    intake_drive(0,0);


    move_forward(12, -0.3);

    intake_drive(0,0);
    arm_drive(4);
    pros::delay(200);

    move_forward(26, 0.2);
    intake_drive(80,80);
    pros::delay(750);
    intake_drive(0,0);

    move_forward(10, -0.3);
    pros::delay(100);

    arm_drive(0);

    // ----------> insert turning and stacking more cubes

    move_forward(18,-0.4);
    chassis->turnAngle(302_deg);

    intake_drive(-120,-120);

    move_forward(34,.4);
    pros::delay(200);

    intake_drive(200,200);
    pros::delay(100);
    intake_drive(0,0);


    move_forward(14, -0.3);

    intake_drive(0,0);
    arm_drive(2);
    pros::delay(200);

    move_forward(12, 0.2);
    intake_drive(100,100);
    pros::delay(750);
    intake_drive(0,0);

    move_forward(10, -0.3);
    pros::delay(100);
    arm_drive(0);
    chassis->stop();
    */
//    gyroTurnR(90, 200);
   turnP(90, 127, true);
}