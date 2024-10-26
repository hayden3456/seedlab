/**
  * @file communication_module.cpp
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Source file for the communication module
  */

#include "communication_module.h"
#include "defs.h"

// Create communication module pointer and set to null pointer
comm_mod* comm_mod::comm_mod_ptr = nullptr;

/**
  * @brief Communication module constructor
  */
comm_mod::comm_mod()
{
  
}

/**
  * @brief Communication module destructor
  */
comm_mod::~comm_mod()
{

}

/**
  * @brief Gets instance of communication module
  *
  * @return Communication module pointer
  */
comm_mod* comm_mod::get_instance()
{
  // Check if communication module is instantiated
  if(comm_mod_ptr == nullptr)
  {
    // Create new instance of communication module
    comm_mod_ptr = new comm_mod();
  }

  return comm_mod_ptr;
}

/**
  * @brief Deallocates memory used for communication module
  */
void comm_mod::clean_up()
{
  // Check if communication module is instantiated
  if(comm_mod_ptr == nullptr)
  {
    // End cleanup if no communication mdoule exists
    return;
  }

  // Delete communication module instance
  delete comm_mod_ptr;

  // Set communication module pointer to null pointer
  comm_mod_ptr = nullptr;
}

/**
  * @brief Prints parameters over serial
  */
void comm_mod::print_params(long current_time)
{
  // Print parameters to console (if enabled)
  // Current program time
  Serial.print(float(current_time) / 1000);

  // Left wheel postion
  if(POSITION_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_left_wheel_pos());
  }

  // Left wheel desired postion
  if(DESIRED_POSITION_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_left_desired_pos());
  }

  // Left wheel velocity
  if(VELOCITY_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_left_wheel_vel());
  }

  // Left motor pwm
  if(PWM_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_left_pwm());
  }

  // Right wheel postion
  if(POSITION_PRINT && RIGHT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_right_wheel_pos());
  }

  // Right wheel desired postion
  if(DESIRED_POSITION_PRINT && LEFT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_right_desired_pos());
  }

  // Right wheel velocity
  if(VELOCITY_PRINT && RIGHT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(loc_mod::get_instance()->get_right_wheel_vel());
  }

  // Right motor pwm
  if(PWM_PRINT && RIGHT_MOTOR_PRINT)
  {
    Serial.print(",");
    Serial.print(cont_mod::get_instance()->get_right_pwm());
  }
  Serial.println();

  // Print "STOP" if MATLAB enabled and program time is equal to MATLAB_TIME
  if(MATLAB_COM && current_time == MATLAB_TIME)
  {
    Serial.println("STOP");
  }
}