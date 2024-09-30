#include "localization_mod.h"

#include "defs.h"

localization_mod::localization_mod()
{
  &left_wheel_pos = wheel_position_mod(LEFT_MOTOR_INPUT_A, LEFT_MOTOR_INPUT_B);
  &right_wheel_pos = wheel_position_mod(RIGHT_MOTOR_INPUT_A, RIGHT_MOTOR_INPUT_B);
}

float localization_mod::get_wheel_pos(int wheel)
{
  if(wheel == 0)
  {
    return left_wheel_pos->get_wheel_position_rads();
  }
  else if(wheel == 1)
  {
    return left_wheel_pos->get_wheel_position_rads();
  }
  else
  {
    return 0;
  }
}