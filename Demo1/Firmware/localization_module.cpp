#include "localization_module.h"
#include "defs.h"

loc_mod* loc_mod::loc_mod_ptr = nullptr;

// This is the Contructor Function
loc_mod::loc_mod()
{
  left_enc = new Encoder(LEFT_MOTOR_INPUT_A, LEFT_MOTOR_INPUT_B);
  right_enc = new Encoder(RIGHT_MOTOR_INPUT_A, RIGHT_MOTOR_INPUT_B);
}

// This is the Destructor Function
loc_mod::~loc_mod()
{
  delete left_enc;
  delete right_enc;

  left_enc = nullptr;
  right_enc = nullptr;
}

// This function creates a version of the class that will be called for the different left and right motors
loc_mod* loc_mod::get_instance()
{
  if(loc_mod_ptr == nullptr)
  {
    loc_mod_ptr = new loc_mod();
  }

  return loc_mod_ptr;
}

// This resets the pointer to a null pointer to keep from breaking things
void loc_mod::clean_up()
{
  if(loc_mod_ptr == nullptr)
  {
    return;
  }
  delete loc_mod_ptr;
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
  left_vel = get_left_wheel_vel() - 2.5;
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
