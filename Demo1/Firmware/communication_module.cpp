#include "communication_module.h"
#include "defs.h"

comm_mod* comm_mod::comm_mod_ptr = nullptr;

// This is the Contructor Function
comm_mod::comm_mod()
{
  
}
// This is the Destructor Function
comm_mod::~comm_mod()
{

}
// This function creates a version of the class that will be called for the different left and right motors
comm_mod* comm_mod::get_instance()
{
  if(comm_mod_ptr == nullptr)
  {
    comm_mod_ptr = new comm_mod();
  }

  return comm_mod_ptr;
}

// This resets the pointer to a null pointer to keep from breaking things
void comm_mod::clean_up()
{
  if(comm_mod_ptr == nullptr)
  {
    return;
  }
  delete comm_mod_ptr;
  comm_mod_ptr = nullptr;
}

/**
  * @brief Prints the different positions, desired positions, PWM's and velocities for each motor to Serial
  *
  * @return [No return]
  */
void comm_mod::print_params(long current_time)
{
  // Print wheel params to console
  Serial.print(float(current_time) / 1000);

  if(POSITION_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_left_wheel_pos());
  }
  if(DESIRED_POSITION_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_left_desired_pos());
  }
  if(VELOCITY_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_left_wheel_vel());
  }
  if(PWM_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_left_pwm());
  }
  if(POSITION_PRINT && RIGHT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_right_wheel_pos());
  }
  if(DESIRED_POSITION_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_right_desired_pos());
  }
  if(VELOCITY_PRINT && RIGHT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_right_wheel_vel());
  }
  if(PWM_PRINT && RIGHT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_right_pwm());
  }
  Serial.println();

  if(MATLAB_COM && current_time == MATLAB_TIME)
  {
    Serial.println("STOP");
  }
}
