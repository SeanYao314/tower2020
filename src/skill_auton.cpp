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
    vector<vector<int>> data_points = {
            {0,0,0,0,0,-10441,0},
            {1,0,0,0,0,-6222,-4540},
            {2,0,0,0,0,-2673,-13000},
            {3,0,0,0,6,-80,-16000},
            {4,0,0,0,6,-6,-16000},
            {5,0,0,0,0,6,-16000},
            {6,0,0,0,6,6,-16000},
            {7,0,0,0,0,6,-9813},
            {8,0,0,0,6,0,-4885},
            {9,0,0,0,-6,18,-8008},
            {10,0,0,0,6,6,-8408},
            {11,0,0,0,6,6,-7749},
            {12,0,0,0,-6,-6,-6308},
            {13,0,0,0,0,6,-3708},
            {14,0,0,0,6,0,-290},
            {15,0,0,0,6,0,733},
            {16,0,0,0,0,0,936},
            {17,0,0,0,-6,0,1115},
            {18,0,0,0,6,0,1060},
            {19,0,0,0,0,0,838},
            {20,0,0,0,-6,6,727},
            {21,0,0,0,-6,6,376},
            {22,0,0,0,6,6,6}
        };
    replay_dp(data_points);
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
        {32,4000,4000,-9000,9000,0,0},
        {190,0,0,0,0,0,0}
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
        {304,0,6,1602,-1768,-5599,0},
        {305,0,-6,2174,-2199,-11852,6},
        {306,0,-6,2261,-2211,-11993,0},
        {307,6,0,2285,-2131,-11913,0},
        {308,0,-6,2248,-2107,-11938,0},
        {309,0,0,2205,-2150,-11907,0},
        {310,0,0,2187,-2150,-11913,0},
        {311,0,0,2131,-2101,-11932,0},
        {312,0,0,2113,-2187,-12067,0},
        {313,0,6,2125,-2298,-12030,0},
        {314,6,0,2076,-2224,-12061,6},
        {315,0,0,2088,-2230,-12036,6},
        {316,6,0,2125,-2224,-12067,0},
        {317,-6,-6,2279,-2187,-11993,-6},
        {318,0,0,2421,-2131,-11919,0},
        {319,0,0,2273,-2125,-11907,-6},
        {320,0,6,2156,-2113,-11956,0},
        {321,0,0,2057,-2045,-12012,6},
        {322,0,0,2020,-2045,-12080,6},
        {323,0,0,2057,-2107,-12129,6},
        {324,6,-6,2051,-2193,-12006,6},
        {325,6,0,2027,-2131,-11987,6},
        {326,6,0,2014,-2039,-12055,-6},
        {327,6,6,2002,-2014,-12036,-6},
        {329,0,0,2014,-2002,-12055,0},
        {330,0,6,2002,-2033,3825,6},
        {331,6,0,1990,-2014,-8076,-6},
        {332,6,0,2008,-2033,-2526,0},
        {333,0,881,2008,-2002,-5821,0},
        {334,0,893,2014,-2008,-4928,6},
        {335,6,819,2002,-1996,-5926,0},
        {336,0,825,2033,-2002,-5901,6},
        {337,6,813,2051,-2002,-5889,-6},
        {338,0,819,2014,-1990,-5907,6},
        {339,0,1183,2020,-1990,-5427,0},
        {340,0,1189,2014,-2008,-5488,-6},
        {341,881,1552,2027,-2014,-4996,6},
        {342,893,1756,1996,-2002,-4990,0},
        {343,887,2378,1990,-2002,-4540,6},
        {344,1103,2815,1996,-2002,-4971,-6},
        {345,1417,2914,1996,-1990,-4595,0},
        {346,1577,2914,2008,-2008,-4454,0},
        {347,1608,2821,2008,-2002,-4497,6},
        {348,1552,2803,1996,-2008,-4854,0},
        {349,1565,2834,2008,-2014,-4749,6},
        {350,1552,2901,1977,-2002,-5094,6},
        {351,1552,2895,1977,-1977,-4983,6},
        {352,1651,2797,1983,-1977,-4817,6},
        {353,1651,2901,1977,-1983,-5088,0},
        {354,1651,2901,1983,-1996,-4749,0},
        {355,1534,2803,1990,-1996,-4762,0},
        {356,1392,2803,1983,-1965,-4694,0},
        {357,1269,2803,1990,-1971,-4817,6},
        {358,924,2809,1977,-2002,-4694,0},
        {359,678,2809,1990,-1983,-4768,0},
        {360,-12,2809,1996,-1983,-4571,0},
        {361,-123,2797,1977,-1983,-4484,0},
        {362,0,2790,1977,-1983,-4700,6},
        {363,6,2797,1983,-1983,-4762,12},
        {364,0,2797,2002,-1977,-4614,0},
        {365,0,2803,1983,-1971,-4879,6},
        {366,6,1756,1983,-1996,-4953,6},
        {367,-99,-604,-296,573,-1731,6},
        {368,-579,-382,-25,203,-561,6},
        {369,-536,-702,0,-12,-12,6},
        {370,-634,-561,-6,6,-6,0},
        {371,-616,-567,10601,-10589,0,0},
        {372,-561,-715,9911,-8987,0,0},
        {373,-1195,-1047,7022,-9622,0,-12},
        {374,-906,-788,9092,-11420,6,-172},
        {375,-721,-690,11507,-11427,6,-55},
        {376,-308,-191,6449,-228,0,0},
        {377,31,0,-739,665,0,-6},
        {378,6,0,-739,246,6,0},
        {379,-6,0,-579,-37,6,0},
        {380,-6,0,-154,12,6,0},
        {381,0,-6,0,0,6,6}
        };
    replay_dp(data_points);
}

void turning(int degrees) {
    const double pi = 3.14159265358979323846;
    const double chassis_power = 0.2;
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

void skill_auton() {
    raise_the_arm_and_release_anti_tip();
    move_forward_take_9_cubes();
    pros::delay(50);

    // turning(-45);
    // pros::delay(50);

    // move_forward(30, 0.5);
    // pros::delay(50);
    // move_straight(72, 0.5);
    // turning(-45);
    // stack_9_cubes();
}