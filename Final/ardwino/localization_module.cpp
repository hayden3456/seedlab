/**
  * @file localization_module.cpp
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Source file for the localization module
  */

#include "localization_module.h"
#include "defs.h"

// Create localization module pointer and set to null pointer
loc_mod* loc_mod::loc_mod_ptr = nullptr;

/**
  * @brief Localization module constructor
  */
loc_mod::loc_mod()
{
  // Create new encoder instances and assign them to encoder pointer members
  left_enc = new Encoder(LEFT_MOTOR_INPUT_A, LEFT_MOTOR_INPUT_B);
  right_enc = new Encoder(RIGHT_MOTOR_INPUT_A, RIGHT_MOTOR_INPUT_B);
}

/**
  * @brief Localization module destructor
  */
loc_mod::~loc_mod()
{
  // Deallocate memory storing encoder objects
  delete left_enc;
  delete right_enc;

  // Set encoder pointer members to null pointers
  left_enc = nullptr;
  right_enc = nullptr;
}

/**
  * @brief Gets instance of localization module
  *
  * @return Localization module pointer
  */
loc_mod* loc_mod::get_instance()
{
  // Check if localization module is instantiated
  if(loc_mod_ptr == nullptr)
  {
    // Create new instance of localization module
    loc_mod_ptr = new loc_mod();
  }

  return loc_mod_ptr;
}

/**
  * @brief Deallocates memory used for localization module
  */
void loc_mod::clean_up()
{
  // Check if localization module is instantiated
  if(loc_mod_ptr == nullptr)
  {
    // End cleanup if no localization mdoule exists
    return;
  }

  // Delete localization module instance
  delete loc_mod_ptr;

  // Set localization module pointer to null pointer
  loc_mod_ptr = nullptr;
}

/**
  * @brief Gets absolute wheel position
  *
  * @param enc Wheel encoder object pointer
  * @return Absolute wheel position (rad)
  */
float loc_mod::get_wheel_pos(Encoder *enc)
{
  // Calculate and return absolute wheel position (rads)
  return float(enc->read()) * ((2 * PI) / 3200);
}

/**
  * @brief Gets wheel velocity
  *
  * @param current_pos Current wheel position
  * @param prev_pos Previous wheel position
  * @return Measured wheel speed (rad/s)
  */
float loc_mod::get_wheel_vel(float current_pos, float prev_pos)
{
  // Calculate and return wheel velocity (rad/s)
  return (current_pos - prev_pos) / (float(LOCALIZATION_UPDATE_PERIOD) / 1000);
}

/**
  * @brief Updates previous and current wheel positions
  */
void loc_mod::update_wheel_pos()
{
  left_prev_pos = left_current_pos;
  right_prev_pos = right_current_pos;

  left_current_pos = get_left_wheel_pos();
  right_current_pos = get_right_wheel_pos();
}

/**
  * @brief Updates wheel velocities
  */
void loc_mod::update_wheel_vels()
{
  left_vel = get_left_wheel_vel();
  right_vel = get_right_wheel_vel();
}

/**
  * @brief Gets left absolute wheel position
  *
  * @return Left absolute wheel position (rad)
  */
float loc_mod::get_left_wheel_pos()
{
  return get_wheel_pos(left_enc);
}

/**
  * @brief Gets right absolute wheel position
  *
  * @return Right absolute wheel position (rad)
  */
float loc_mod::get_right_wheel_pos()
{
  return -get_wheel_pos(right_enc);
}

/**
  * @brief Gets left wheel velocity
  *
  * @return Left wheel velocity (rad/s)
  */
float loc_mod::get_left_wheel_vel()
{
  return get_wheel_vel(left_current_pos, left_prev_pos);
}

/**
  * @brief Gets right wheel velocity
  *
  * @return Right wheel velocity (rad/s)
  */
float loc_mod::get_right_wheel_vel()
{
  return get_wheel_vel(right_current_pos, right_prev_pos);
}
