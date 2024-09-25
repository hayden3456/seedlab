#include "voltage_mod.h"
#include "localization_mod.h"
#include "SEED_Lab_PID_controller.h"

using namespace std;

int Kp = 10;
float vel_refresh_interval = 0.01;

void velocity_control::left_voltage_set(float desired_left_vel, float measured_left_vel) {
    float left_vel_error = desired_left_vel - measured_left_vel;
    left_voltage = Kp * left_vel_error;
    if (left_voltage < 0) {
        left_dir = 0;
        left_voltage = -left_voltage;
    } 
    else { left_dir = 1; }

}

void velocity_control::left_velocity_set(float left_pos = get_wheel_pos(0), float left_prev_pos) {
    measured_left_vel = float((((2*3.14159) / 3200) * (left_pos - left_prev_pos)) / vel_refresh_interval);
    left_prev_pos = left_pos;
}

void velocity_control::right_voltage_set(float desired_right_vel, float measured_right_vel) {
    float right_vel_error = desired_right_vel - measured_right_vel;
    right_voltage = Kp * right_vel_error;
    if (right_voltage < 0) {
        right_dir = 0;
        right_voltage = -right_voltage;
    } 
    else { right_dir = 1; }
}

void velocity_control::right_velocity_set(float right_pos = get_wheel_pos(1), float right_prev_pos) {
    measured_right_vel = float((((2*3.14159) / 3200) * (right_pos - right_prev_pos)) / vel_refresh_interval);
    right_prev_pos = right_pos;
}


float velocity_control::left_voltage_get() {
    return left_voltage;
}

float velocity_control::right_voltage_get() {
    return right_voltage;
}

float velocity_control::measured_left_vel_get() {
    return measured_left_vel;
}

float velocity_control::measured_right_vel_get() {
    return measured_right_vel;
}

float velocity_control::left_dir_get() {
    return left_dir;
}

float velocity_control::right_dir_get() {
    return right_dir;
}

