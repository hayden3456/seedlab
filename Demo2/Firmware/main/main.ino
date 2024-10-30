/**
  * @file main.ino
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Main arduino file for running firmware
  */
#include "defs.h"
#include "localization_module.h"
#include "control_module.h"
#include "communication_module.h"

#include <Wire.h>

// Functions
// Control
void set_motor_vel(int pwm, int dir_pin, int speed_pin);

void receiveEvent(int bytesReceived);

// Variables
// Scheduler
long comm_mod_service_time = 0;
long loc_mod_service_time = 0;
long cont_mod_service_time = 0;

// Control
float desired_rotation_angle = 0;
float desired_forward_distance = 0;

long time_stopped = 0;

bool rotation_complete = false;
bool rotation_written = false;
bool forward_written = false;

//Communication
long console_service_time = 0;

volatile uint8_t offset = 0;

int cmd_arr[2];
int new_cmd = 0;
int count = 0;

void setup() {
  // Initialize serial port
  Serial.begin(BAUD_RATE);
  
  // Initizlize i2c communication
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
 
  // Configure pin modes
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_SPEED, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED, OUTPUT);

  // Enable motor controller
  digitalWrite(MOTOR_ENABLE, 1);

  // Print "GO" if MATLAB enabled
  if(MATLAB_COM)
  {
    Serial.println("GO");
  }
}

void loop() {
  // Update current time
  long current_time = millis();

  // Write rotation command
  if(!rotation_complete && !rotation_written)
  {
    // Write wheel positions
    cont_mod::get_instance()->set_left_desired_pos(loc_mod::get_instance()->get_left_wheel_pos() + (desired_rotation_angle * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));
    cont_mod::get_instance()->set_right_desired_pos(loc_mod::get_instance()->get_right_wheel_pos() - (desired_rotation_angle * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));

    // Set rotation flag
    rotation_written = true;
  }

  // Write forward command (once rotation complete)
  if(rotation_complete && !forward_written)
  {
    // Write wheel positions
    cont_mod::get_instance()->set_left_desired_pos(loc_mod::get_instance()->get_left_wheel_pos() + (desired_forward_distance * FEET_TO_CM) / (WHEEL_RADIUS));
    cont_mod::get_instance()->set_right_desired_pos(loc_mod::get_instance()->get_right_wheel_pos() + (desired_forward_distance * FEET_TO_CM) / (WHEEL_RADIUS));

    // Set forward flag
    forward_written = true;
  }

  // Console event (check if service time elapsed)
  if(current_time >= (comm_mod_service_time + CONSOLE_REF))
  {
    // Reset service time
    comm_mod_service_time = current_time;

    // Print wheel params to console
    comm_mod::get_instance()->print_params(comm_mod_service_time);
  }

  // Localization module update event (check if service time elapsed)
  if(current_time >= (loc_mod_service_time + LOCALIZATION_UPDATE_PERIOD))
  {
    // Reset service time
    loc_mod_service_time = current_time;

    // Update wheel positions
    loc_mod::get_instance()->update_wheel_pos();

    // Update wheel velocities
    loc_mod::get_instance()->update_wheel_vels();
  }

  // Control module update event (check if service time elapsed)
  if(current_time >= (cont_mod_service_time + CONTROL_UPDATE_PERIOD))
  {
    // Reset service time
    cont_mod_service_time = current_time;

    // Update desired velocity (position controller)
    cont_mod::get_instance()->update_desired_vels();

    // Update motor velocities (velocity controller)
    cont_mod::get_instance()->update_motor_pwms();
    
    // Set motor pwm
    set_motor_vel(cont_mod::get_instance()->get_left_pwm(), LEFT_MOTOR_DIRECTION, LEFT_MOTOR_SPEED);
    set_motor_vel(cont_mod::get_instance()->get_right_pwm(), RIGHT_MOTOR_DIRECTION, RIGHT_MOTOR_SPEED);

    // Check if wheels are stopped
    if(loc_mod::get_instance()->get_left_wheel_vel() < 0.01 && loc_mod::get_instance()->get_right_wheel_vel() < 0.01)
    {
      // Increment time stopped
      time_stopped += CONTROL_UPDATE_PERIOD;
    }
    else
    {
      // Reset time stopped
      time_stopped = 0;
    }
  }

  // Check if time stopped elapsed waiting time (indicates rotation complete)
  if(time_stopped > WAIT_TIME && !rotation_complete)
  {
    // Set rotation complete flag
    rotation_complete = true;
  }
}

/**
  * @brief Sets motor velocities by setting speed and direction pin volages.
  *
  * @param pwm Signed pwm value containing speed and direction information
  * @param dir_pin Direction pin ID
  * @param speed_pin Speed pin ID
  */
void set_motor_vel(int pwm, int dir_pin, int speed_pin)
{
  // Check pwm sign
  if(pwm > 0)
  {
    // Write direction and speed pins
    digitalWrite(dir_pin, 1);
    analogWrite(speed_pin, abs(pwm));
  }
  else
  {
    // Write direction and speed pins
    digitalWrite(dir_pin, 0);
    analogWrite(speed_pin, abs(pwm));
  }
}

/**
  * @brief i2c communication ISR
  *
  * @param bytesRecieved is a variable for the information in
  */
void receiveEvent(int bytesReceived) {
  offset = Wire.read();

  // Loop through all received bytes
  while (Wire.available()) 
  {
     // Read each byte
    int x = Wire.read();

    // Write byte to command array
    cmd_arr[count++] = x;

    // Reset count
    if (count >= 2)
    {
      count = 0;
    } 
  }

  // Set new command flag
  new_cmd = 1;
}
