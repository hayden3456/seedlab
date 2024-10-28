/**
  * @file localization_module.h
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Header file for the localization module
  */

#ifndef localization_module_h
#define localization_module_h

#include <Encoder.h>

class loc_mod
{
  private:
    static loc_mod* loc_mod_ptr;

    Encoder* left_enc = nullptr;
    Encoder* right_enc = nullptr;

    float left_current_pos = 0;
    float right_current_pos = 0;
    float left_prev_pos = 0;
    float right_prev_pos = 0;
    float left_vel = 0;
    float right_vel = 0;

    loc_mod();
    ~loc_mod();

    float get_wheel_pos(Encoder *enc);
    float get_wheel_vel(float current_pos, float prev_pos);

  public:
    static loc_mod* get_instance();
    static void clean_up();

    void update_wheel_pos();
    void update_wheel_vels();

    float get_left_wheel_pos();
    float get_right_wheel_pos();
    float get_left_wheel_vel();
    float get_right_wheel_vel();
};
#endif