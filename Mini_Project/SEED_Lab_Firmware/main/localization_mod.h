#ifndef localization_mod_h
#define localization_mod_h

#include "wheel_position.h"

class localization_mod
{
  private:
    wheel_position_mod *left_wheel_pos;
    wheel_position_mod *right_wheel_pos;

  public:
    localization_mod();

    float get_wheel_pos(int wheel);
}

#endif