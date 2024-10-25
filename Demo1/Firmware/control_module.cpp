#include "control_module.h"
#include "defs.h"

cont_mod* cont_mod::cont_mod_ptr = nullptr;
// This is the Constructor Function
cont_mod::cont_mod()
{
  
}
// This is the Destructor Function
cont_mod::~cont_mod()
{

}
// This function creates the pointer for the class for each different motor
cont_mod* cont_mod::get_instance()
{
  if(cont_mod_ptr == nullptr)
  {
    cont_mod_ptr = new cont_mod();
  }

  return cont_mod_ptr;
}
// This function deletes the pointers for the class for each different motor
void cont_mod::clean_up()
{
  if(cont_mod_ptr == nullptr)
  {
    return;
  }
  delete cont_mod_ptr;
  cont_mod_ptr = nullptr;
}

/**
  * @brief Gets motor pwm corresponding to desired velocity
  *
  * @param desired_vel Desired wheel velocity
  * @param measured_vel Measured wheel velocity
  * @return Motor pwm
  */
int cont_mod::get_motor_pwm(float desired_vel, float measured_vel, float kp, float *integral_vel, int output_cap)
{
  // Calculate velocity error
  float vel_error = desired_vel - measured_vel;

  // Integrate velocity error
  if(*integral_vel + kp * vel_error < output_cap)
  {
    *integral_vel += kp * vel_error;
  }
  else
  {
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
  * @brief Sets desired left wheel position
  *
  * @return Set desired Left wheel position (rad)
  */
void cont_mod::set_left_desired_pos(float pos)
{
  left_desired_pos = pos;
}

/**
  * @brief Sets right desired position (rad)
  *
  * @return Set desired Right wheel position (rad)
  */
void cont_mod::set_right_desired_pos(float pos)
{
  right_desired_pos = pos;
}

/**
  * @brief Sets left wheel velocity
  *
  * @return Set desired Left wheel velocity (rad/sec)
  */
void cont_mod::set_left_desired_vel(float vel)
{
  left_desired_vel = vel;
}

/**
  * @brief Sets right wheel velocity
  *
  * @return Set desired Right wheel velocity (rad/sec)
  */
void cont_mod::set_right_desired_vel(float vel)
{
  right_desired_vel = vel;
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
  * @brief Gets left wheel velocity
  *
  * @return Left wheel position (rad)
  */
float cont_mod::get_left_desired_pos()
{
  return left_desired_pos;
}

/**
  * @brief Gets right wheel velocity
  *
  * @return Right wheel position (rad)
  */
float cont_mod::get_right_desired_pos()
{
  return right_desired_pos;
}
