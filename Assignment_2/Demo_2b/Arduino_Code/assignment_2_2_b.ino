#include <Encoder.h>
#include <math.h>

// File Name: demo_2_b.ino
// Author: Luca Ciancanelli
// EENG350 - Section B
//
// Description:
// This arduino script is used to implement position tracking
// of a SEED lab robot. Time, x position, y position and angle 
// of the robot is printed to the console.

// Function declarations
void console_print(long current_time);
void update_position();

float calc_motor_vel(long curr_enc_pos, long prev_enc_pos);

// Constants
// Pin designators
const int Left_Motor_A_Input = 2;
const int Right_Motor_A_Input = 3;
const int Left_Motor_B_Input = 5;
const int Right_Motor_B_Input = 6;

// Refresh intervals (ms)
const int console_refresh_interval = 10;
const int velocity_refresh_interval = 10;
const int position_refresh_interval = 10;

// Global variables
// Scheduler time tracker
long console_prev_time = 0;
long velocity_prev_time = 0;
long position_prev_time = 0;

// Position variables
float x_pos = 0;
float y_pos = 0;
float phi = 0;

// Velocity variables
float left_motor_vel = 0;
float right_motor_vel = 0;
long left_prev_pos = 0;
long right_prev_pos = 0;

// Encoder objects
Encoder left_encoder(Left_Motor_A_Input, Left_Motor_B_Input);
Encoder right_encoder(Right_Motor_A_Input, Right_Motor_B_Input);

void setup() {
  // Initialize COM port at 115200 baud
  Serial.begin(115200);

  // Start MATLAB data collection
  Serial.println("GO");
}

void loop() {
  // Update current time
  long current_time = millis();

  // Update motor velocities
  if(current_time >= (velocity_prev_time + velocity_refresh_interval))
  {
    // Read current encoder positions
    long left_curr_pos = left_encoder.read();
    long right_curr_pos = right_encoder.read();

    // Calculate velocities
    left_motor_vel = calc_motor_vel(left_curr_pos, left_prev_pos);
    right_motor_vel = -calc_motor_vel(right_curr_pos, right_prev_pos);

    // Update tracking variables
    left_prev_pos = left_curr_pos;
    right_prev_pos = right_curr_pos;
    velocity_prev_time = current_time;
  }

  // Update motor velocities
  if(current_time >= (position_prev_time + position_refresh_interval))
  {
    // Run position update
    update_postion();

    // Update tracking variables
    position_prev_time = current_time;
  }

  // Print variables to console
  if(current_time >= (console_prev_time + console_refresh_interval))
  {
    // Print to console
    console_print(current_time);

    // Update tracking variables
    console_prev_time = current_time;
  }

  if(current_time == 10000)
  {
    // Stop MATLAB data collection
    Serial.println("STOP");
  }
}

void update_postion()
{
  // x = x + (delta t) * cos(phi) * (v_r + v_l) / 2
  x_pos += (float(position_refresh_interval) / 1000) * cos(phi) * ((right_motor_vel * 0.075) + (left_motor_vel * 0.075)) / 2;

  // y = y + (delta t) * sin(phi) * (v_r + v_l) / 2
  y_pos += (float(position_refresh_interval) / 1000) * sin(phi) * ((right_motor_vel * 0.075) + (left_motor_vel * 0.075)) / 2;

  // phi = phi + (delta t) * (1/b) * (v_r - v_l)
  phi += (float(position_refresh_interval) / 1000) * (1 / 0.35) * ((right_motor_vel * 0.075) - (left_motor_vel * 0.075));
}

void console_print(long current_time)
{
  // Print current runtime (s)
  Serial.print(float(console_prev_time) / 1000);

  // Print voltage applied to left motor
  Serial.print(",");
  Serial.print(x_pos);

  // Print left motor velocity
  Serial.print(",");
  Serial.print(y_pos);

  // Print left motor velocity
  Serial.print(",");
  Serial.println(phi);
}

float calc_motor_vel(long curr_enc_pos, long prev_enc_pos)
{
  // Return mid point velocity estimation - (conversion factor) * (change in postion) * (time elapsed)
  return ((2 * 3.1415) / 3200) * (curr_enc_pos - prev_enc_pos) / (float(velocity_refresh_interval) / 1000);
}