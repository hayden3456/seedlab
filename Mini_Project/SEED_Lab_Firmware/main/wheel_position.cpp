#include "wheel_position.h"

wheel_position_mod::wheel_position_mod(int pin_1, int pin_2)
{
  &wheel_encoder = Encoder(pin_1, pin_2)
}

float wheel_position_mod::get_wheel_position_rads()
{
  return float(wheel_encoder->read()) * ((2 * 3.14159) / 3200);
}