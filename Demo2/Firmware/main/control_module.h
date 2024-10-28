/**
  * @file control_module.h
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Header file for the localization module
  */

#ifndef control_module_h
#define control_module_h

#include "localization_module.h"

class cont_mod
{
  private:
    static cont_mod* cont_mod_ptr;

    int left_motor_pwm = 0;
    int right_motor_pwm = 0;
    float left_motor_comp = 1;

    float left_desired_vel = 0;
    float right_desired_vel = 0;
    float left_vel_integral = 0;
    float right_vel_integral = 0;

    float left_desired_pos = 0;
    float right_desired_pos = 0;
    float left_pos_integral = 0;
    float right_pos_integral = 0;

    cont_mod();
    ~cont_mod();

    int get_motor_pwm(float desired_vel, float measured_vel, float kp, float *integral_vel, int output_cap);
    float get_desired_vel(float desired_pos, float current_pos, float kp, float ki, float *integral_error);

  public:
    static void clean_up();
    static cont_mod* get_instance();

    void update_desired_vels();
    void update_motor_pwms();

    void set_left_desired_pos(float pos);
    void set_right_desired_pos(float pos);
    void set_left_desired_vel(float vel);
    void set_right_desired_vel(float vel);
    
    int get_left_pwm();
    int get_right_pwm();
    float get_left_desired_pos();
    float get_right_desired_pos();
};

#endif