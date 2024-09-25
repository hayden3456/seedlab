/* This the the PID controller module for the mini project*/

#include "SEED_Lab_PID_controller.h"
#include "localization_mod.h"
#include "defs.h"

using namespace std;

float Kp_pos = 1, Ki_pos = 1;
int desired_Ts_ms = 10;

float PID_control::get_right_vel(){
    return desired_right_vel;
}

void PID_control::set_right_vel(long *desired_pos, float measured_pos = get_wheel_pos(1), float *desired_right_vel){
  long pos_error  = desired_pos- measured_pos;
  long integral_error = integral_error + pos_error*((float)desired_Ts_ms /1000);
  *desired_right_vel = Kp_pos * pos_error + Ki_pos * integral_error;
}