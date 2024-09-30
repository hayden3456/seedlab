#include "voltage_mod.h"
#include "localization_mod.h"
#include "SEED_Lab_PID_controller.h"

using namespace std;

int Kp = 10;
float vel_refresh_interval = 0.01;

void velocity_control::set_left_voltage(float desired_left_vel = get_left_vel(), float measured_left_vel = get_measured_vel(0)) {
    float left_vel_error = desired_left_vel - measured_left_vel;
    left_voltage = Kp * left_vel_error;
    if (left_voltage < 0) {
        left_dir = 0;
        left_voltage = -left_voltage;
    } 
    else { left_dir = 1; }
}

void velocity_control::set_right_voltage(float desired_right_vel = get_right_vel(), float measured_right_vel = get_measured_vel(1)) {
    float right_vel_error = desired_right_vel - measured_right_vel;
    right_voltage = Kp * right_vel_error;
    if (right_voltage < 0) {
        right_dir = 0;
        right_voltage = -right_voltage;
    } 
    else { right_dir = 1; }
}

float velocity_control::set_measured_left_vel(float left_pos = get_wheel_pos(0), float left_prev_pos) {
    measured_left_vel = float((((2*3.14159) / 3200) * (left_pos - left_prev_pos)) / vel_refresh_interval);
    left_prev_pos = left_pos;
}

void velocity_control::set_measured_right_vel(float right_pos = get_wheel_pos(1), float right_prev_pos) {
    measured_right_vel = float((((2*3.14159) / 3200) * (right_pos - right_prev_pos)) / vel_refresh_interval);
    right_prev_pos = right_pos;
}

float velocity_control::get_voltage(int wheel) {
    if(wheel == 0) {
        return left_voltage;
    }
    else if(wheel == 1) {
        return right_voltage;
    }
    else {
        return 0;
    }
}

float velocity_control::get_measured_vel(int wheel) {
    if(wheel == 0) {
        set_left_velocity();
        return measured_left_vel;
    }
    else if(wheel == 1) {
        set_right_velocity();
        return measured_right_vel;
    }
    else {
        return 0;
    }
}

float velocity_control::get_dir(int wheel) {
    if(wheel == 0) {
        return left_dir;
    }
    else if(wheel == 1) {
        return right_dir;
    }
    else {
        return 0;
    }
}


float velocity_control::right_dir_get() {
    return right_dir;
}

