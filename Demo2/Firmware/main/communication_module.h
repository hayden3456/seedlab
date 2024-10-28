/**
  * @file communication_module.h
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Header file for the communication module
  */

#ifndef communication_module_h
#define communication_module_h

#include "localization_module.h"
#include "control_module.h"

class comm_mod
{
  private:
    static comm_mod* comm_mod_ptr;

    comm_mod();
    ~comm_mod();

  public:
    static comm_mod* get_instance();
    static void clean_up();

    void print_params(long current_time);
};

#endif