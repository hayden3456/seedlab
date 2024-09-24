#ifndef wheel_position_h
#define wheel_position_h

#include <Encoder.h>

class wheel_position_mod
{
  private:
    Encoder *wheel_encoder;

  public:
    wheel_position_mod(int pin_1, int pin_2);
    float get_wheel_position_rads();
}

#endif