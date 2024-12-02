/**
  * @file control_module.cpp
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Source file for the localization module
  */

#include "control_module.h"
#include "defs.h"

// Create control module pointer and set to null pointer
cont_mod* cont_mod::cont_mod_ptr = nullptr;

/**
  * @brief Control module constructor
  */
cont_mod::cont_mod()
{
  
}

/**
  * @brief Control module destructor
  */
cont_mod::~cont_mod()
{

}

/**
  * @brief Gets instance of control module
  *
  * @return Control module pointer
  */
cont_mod* cont_mod::get_instance()
{
  // Check if control module is instantiated
  if(cont_mod_ptr == nullptr)
  {
    // Create new instance of control module
    cont_mod_ptr = new cont_mod();
  }

  return cont_mod_ptr;
}

/**
  * @brief Deallocates memory used for control module
  */
void cont_mod::clean_up()
{
  // Check if control module is instantiated
  if(cont_mod_ptr == nullptr)
  {
    // End cleanup if no control mdoule exists
    return;
  }

  // Delete control module instance
  delete cont_mod_ptr;

  // Set control module pointer to null pointer
  cont_mod_ptr = nullptr;
}

/**
  * @brief Gets motor pwm corresponding to desired velocity
  *
  * @param desired_vel Desired wheel velocity
  * @param measured_vel Measured wheel velocity
  * @param kp Proportional gain
  * @param integral_vel Current velocity integral pointer
  * @param output_cap Velocity cap
  * @return Motor pwm
  */
int cont_mod::get_motor_pwm(float desired_vel, float measured_vel, float kp, float *integral_vel, int output_cap)
{
  // Calculate velocity error
  float vel_error = desired_vel - measured_vel;

  // Integrate velocity error (if velocity cap is not exceeded)
  if(*integral_vel + kp * vel_error < output_cap)
  {
    // Integrate velocity error
    *integral_vel += kp * vel_error;
  }
  else
  {
    // Set velocity to output cap
    *integral_vel = output_cap;
  }
  
  // Return motor pwm
  return *integral_vel;
}

/**
  * @brief Gets desired velocity for a wheel based on desired wheel position
  *
  * @param desired_pos Desired wheel position
  * @param current_pos Current wheel position
  * @param kp Proportional gain
  * @param ki Integral gain
  * @param integral_error Current integrator error pointer
  * @return Desired wheel velocity (rad/s)
  */
float cont_mod::get_desired_vel(float desired_pos, float current_pos, float kp, float ki, float *integral_error)
{
  // Calculate position error
  float pos_error  = desired_pos - current_pos;

  // Limit integral error with anti-windup
  if(*integral_error <= ANTI_WINDUP && *integral_error >= -ANTI_WINDUP)
  {
    // Calculate/update integral error
    *integral_error += pos_error * (float(CONTROL_UPDATE_PERIOD) / 1000);
  }

  // Calculate and return desired velocity
  return kp * pos_error + ki * (*integral_error);
}

/**
  * @brief Updates the desired velocities of the left and right motors
  *
  */
void cont_mod::update_desired_vels()
{
  left_desired_vel = get_desired_vel(left_desired_pos, loc_mod::get_instance()->get_left_wheel_pos(), KP_POS_LEFT, KI_POS_LEFT, &left_pos_integral);
  right_desired_vel = get_desired_vel(right_desired_pos, loc_mod::get_instance()->get_right_wheel_pos(), KP_POS_RIGHT, KI_POS_RIGHT, &right_pos_integral);
}

/**
  * @brief Updates the pwm values of the left and right motors
  *
  */
void cont_mod::update_motor_pwms()
{
  left_motor_pwm = get_motor_pwm(left_desired_vel, loc_mod::get_instance()->get_left_wheel_vel(), KP_VEL_LEFT, &left_vel_integral, OUTPUT_CAP_LEFT);
  right_motor_pwm = get_motor_pwm(right_desired_vel, loc_mod::get_instance()->get_right_wheel_vel(), KP_VEL_RIGHT, &right_vel_integral, OUTPUT_CAP_RIGHT);
}

/**
  * @brief Sets mode of operation (true - find arco marker, false - move based on PI commands)
  */
void cont_mod::set_operation_mode(int mode)
{
  operation_mode = mode;
}

/**
  * @brief Sets left wheel desired position (rad)
  */
void cont_mod::set_left_desired_pos(float pos)
{
  left_desired_pos = pos;
}

/**
  * @brief Sets right wheel desired position (rad)
  */
void cont_mod::set_right_desired_pos(float pos)
{
  right_desired_pos = pos;
}

/**
  * @brief Sets left wheel desired velocity (rad/sec)
  */
void cont_mod::set_left_desired_vel(float vel)
{
  left_desired_vel = vel;
}

/**
  * @brief Sets right wheel desired velocity (rad/sec)
  */
void cont_mod::set_right_desired_vel(float vel)
{
  right_desired_vel = vel;
}

/**
  * @brief Gets operation mode
  *
  * @return Operation mode (bool)
  */
int cont_mod::get_operation_mode()
{
  return operation_mode;
}

/**
  * @brief Gets left wheel pwm
  *
  * @return Left wheel pwm (int)
  */
int cont_mod::get_left_pwm()
{
  return left_motor_pwm;
}

/**
  * @brief Gets right wheel pwm
  *
  * @return Right wheel pwm (int)
  */
int cont_mod::get_right_pwm()
{
  return right_motor_pwm;
}

/**
  * @brief Gets left wheel desired position
  *
  * @return Left wheel desired position (rad)
  */
float cont_mod::get_left_desired_pos()
{
  return left_desired_pos;
}

/**
  * @brief Gets right wheel desired position
  *
  * @return Right wheel desired position (rad)
  */
float cont_mod::get_right_desired_pos()
{
  return right_desired_pos;
}
