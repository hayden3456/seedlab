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

// Communication
void receive_cmd(int bytes_received);

// Variables
// Scheduler
long comm_mod_service_time = 0;
long loc_mod_service_time = 0;
long cont_mod_service_time = 0;

//Communication
int cmd_arr[CMD_PACKET_SIZE];
int cmd_arr_index = 0;

bool new_cmd = false;
bool cmd_written = false;

void setup() {
  // Initialize serial port
  Serial.begin(BAUD_RATE);
  
  // Initizlize i2c communication
  Wire.begin(ARDUINO_ADDRESS);
  Wire.onReceive(receive_cmd);
 
  // Configure pin modes
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_SPEED, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED, OUTPUT);

  // Enable motor controller
  digitalWrite(MOTOR_ENABLE, 1);

  Serial.println("START");

  // Print "GO" if MATLAB enabled
  if(MATLAB_COM)
  {
    Serial.println("GO");
  }
}

void loop() {
  // Update current time
  long current_time = millis();

  // Communication module update event (check if service time elapsed)
  if(current_time >= (comm_mod_service_time + CONSOLE_REF))
  {
    // Reset service time
    comm_mod_service_time = current_time;

    // Check if new command recieved from PI
    if(new_cmd)
    {
      // Reset new command flag
      new_cmd = false;

      // Write commanded operation mode to control module
      cont_mod::get_instance()->set_operation_mode(cmd_arr[0]);

      // Serial.print(cmd_arr[0]);
      // Serial.print(" ");
      // Serial.print(cmd_arr[1]);
      // Serial.print(" ");
      // Serial.print(cmd_arr[2]);
      // Serial.print(" ");
      // Serial.print(cmd_arr[3]);
      // Serial.print(" ");
      // Serial.print(cmd_arr[4]);
      // Serial.print(" ");
      // Serial.print(cmd_arr[5]);
      // Serial.println();
    }

    // Print wheel params to console
    //comm_mod::get_instance()->print_params(comm_mod_service_time);
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

    // Check operation mode
    if(cont_mod::get_instance()->get_operation_mode() == MOVE_MODE && !cmd_written)
    {
      // Format distance and angle from 2 uint8s to a float
      float angle;
      float distance = (cmd_arr[1]) + (float(cmd_arr[2]) / 100);
      
      if(cmd_arr[5] == 0)
      {
        angle = (cmd_arr[3]) + (float(cmd_arr[4]) / 100);
      }
      else
      {
        angle = -(cmd_arr[3]) + (float(cmd_arr[4]) / 100);
      }

      Serial.print(cmd_arr[5]);
      Serial.print(" ");
      Serial.print(angle);
      Serial.print(" ");
      Serial.println(distance);

      // Write wheel positions to rotate to commanded distance and angle
      cont_mod::get_instance()->set_left_desired_pos(loc_mod::get_instance()->get_left_wheel_pos() + (distance * FEET_TO_CM) / (WHEEL_RADIUS) + (angle * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));
      cont_mod::get_instance()->set_right_desired_pos(loc_mod::get_instance()->get_right_wheel_pos() + (distance * FEET_TO_CM) / (WHEEL_RADIUS) - (angle * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));

      // Set command written flag
      cmd_written = true;
    }
    else if(cont_mod::get_instance()->get_operation_mode() == STOP_MODE && !cmd_written) 
    {
      // Write wheel positions to current position to stop 
      cont_mod::get_instance()->set_left_desired_pos(loc_mod::get_instance()->get_left_wheel_pos());
      cont_mod::get_instance()->set_right_desired_pos(loc_mod::get_instance()->get_right_wheel_pos());

      // Set command written flag
      cmd_written = true;
    }
    // else if(cont_mod::get_instance()->get_operation_mode() == TURN_LEFT && !cmd_written) 
    // {
    //   Serial.println("test");
    //   cont_mod::get_instance()->set_left_desired_pos(loc_mod::get_instance()->get_left_wheel_pos() + (90 * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));
    //   cont_mod::get_instance()->set_right_desired_pos(loc_mod::get_instance()->get_right_wheel_pos() - (90 * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));
    //   cmd_written = true;
    // }
    // else if(cont_mod::get_instance()->get_operation_mode() == TURN_RIGHT && !cmd_written) 
    // {
    //   Serial.println("test");
    //   cont_mod::get_instance()->set_left_desired_pos(loc_mod::get_instance()->get_left_wheel_pos() - (90 * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));
    //   cont_mod::get_instance()->set_right_desired_pos(loc_mod::get_instance()->get_right_wheel_pos() + (90 * DEG_TO_RAD * WHEEL_WIDTH) / (2 * WHEEL_RADIUS));
    //   cmd_written = true;
    // }

    // Update desired velocity (position controller)
    cont_mod::get_instance()->update_desired_vels();

    // Update motor velocities (velocity controller)
    cont_mod::get_instance()->update_motor_pwms();
    
    // Set motor pwm
    set_motor_vel(cont_mod::get_instance()->get_left_pwm(), LEFT_MOTOR_DIRECTION, LEFT_MOTOR_SPEED);
    set_motor_vel(cont_mod::get_instance()->get_right_pwm(), RIGHT_MOTOR_DIRECTION, RIGHT_MOTOR_SPEED);
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
  * @brief I2C communication ISR
  *
  * @param bytes_received number of bytes recieved over I2C
  */
void receive_cmd(int bytes_received) {
  // Ignore first byte (no information)
  Wire.read();

  for(uint8_t i = 0; i < CMD_PACKET_SIZE; i++)
  {
    cmd_arr[i] = Wire.read();
  }

  // Set new command flag
  new_cmd = true;

  // Reset command written
  cmd_written = false;

  return;
}
