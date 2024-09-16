#include <Encoder.h>

// File Name: assignment_2_2_a.ino
// Author: Luca Ciancanelli
// EENG350 - Section B
//
// Description:
// This arduino script is used to implement a proportional
// controller for both motors of a SEED lab robot. Desired
// motor speed can be set for both motors in rad/s. Time, 
// voltage, and velocity for both motors is printed to 
// the console.

// Function declarations
void console_print(long current_time);
void left_motor_p();
void right_motor_p();

float calc_motor_vel(long curr_enc_pos, long prev_enc_pos);
float read_battery_voltage();

// Constants
// Pin designators
const int Left_Motor_A_Input = 2;
const int Right_Motor_A_Input = 3;
const int Left_Motor_B_Input = 5;
const int Right_Motor_B_Input = 6;
const int Motor_Enable = 4;
const int Left_Motor_Direction = 7;
const int Right_Motor_Direction = 8;
const int Left_Motor_Speed = 9;
const int Right_Motor_Speed = 10;

// Refresh intervals (ms)
const int console_refresh_interval = 100;
const int velocity_refresh_interval = 10;
const int p_refresh_interval = 10;

// Motor control parameters
const float Kp = 10;

// Motor desired speed (rad/s)
const float left_motor_desired_speed = 3;
const float right_motor_desired_speed = 0;

// Global variables
// Scheduler time tracker
long console_prev_time = 0;
long vel_prev_time = 0;
long p_prev_time = 0;

// Encoder rotation counter
long left_turn_count = 0;
long right_turn_count = 0;

// Velocity variables
float left_motor_vel = 0;
float right_motor_vel = 0;
long left_prev_pos = 0;
long right_prev_pos = 0;

// Motor states
int left_motor_pwm = 0;
int right_motor_pwm = 0;
int left_motor_dir = 0;
int right_motor_dir = 0;

// Encoder objects
Encoder left_encoder(Left_Motor_A_Input, Left_Motor_B_Input);
Encoder right_encoder(Right_Motor_A_Input, Right_Motor_B_Input);

void setup() {
  // Initialize COM port at 115200 baud
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(Motor_Enable, OUTPUT);
  pinMode(Left_Motor_Speed, OUTPUT);
  pinMode(Right_Motor_Speed, OUTPUT);
  pinMode(Left_Motor_Direction, OUTPUT);
  pinMode(Right_Motor_Direction, OUTPUT);
  
  // Enable motor driver
  digitalWrite(Motor_Enable, HIGH);
}

void loop() {
  // Update current time
  long current_time = millis();

  // Update motor velocity
  if(current_time > (vel_prev_time + velocity_refresh_interval))
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
    vel_prev_time = current_time;
  }

  // Update proportional controller
  if(current_time >= (p_prev_time + p_refresh_interval))
  {
    // Run proportional controllers
    left_motor_p();
    right_motor_p();

    // Update tracking variables
    p_prev_time = current_time;
  }

  // Print variables to console
  if(current_time >= (console_prev_time + console_refresh_interval))
  {
    // Run console print
    console_print(current_time);

    // Update tracking variables
    console_prev_time = current_time;
  }
}

void console_print(long current_time)
{
  // Print current runtime (s)
  Serial.print(float(console_prev_time) / 1000);

  // Print voltage applied to left motor
  Serial.print(" ");
  Serial.print(read_battery_voltage() * (float(left_motor_pwm) / 255));

  // Print left motor velocity
  Serial.print(" ");
  Serial.print(left_motor_vel);

  // Print voltage applied to right motor
  Serial.print(" ");
  Serial.print(read_battery_voltage() * (float(right_motor_pwm) / 255));

  // Print left motor velocity
  Serial.print(" ");
  Serial.println(right_motor_vel);
}

void left_motor_p()
{
  // Calculate error, voltage, and pwm
  float speed_error = left_motor_desired_speed - left_motor_vel;
  float voltage = Kp * speed_error;
  int pwm = float(voltage / read_battery_voltage());

  // Add pwm to current pwm
  left_motor_pwm += pwm;

  // Write motor speed and direction to motor controller
  if(left_motor_pwm > 0)
  {
    digitalWrite(Left_Motor_Direction,1);
    analogWrite(Left_Motor_Speed, min(255,abs(left_motor_pwm)));
  }
  else
  {
    digitalWrite(Left_Motor_Direction,0);
    analogWrite(Left_Motor_Speed, min(255,abs(left_motor_pwm)));
  }
}

void right_motor_p()
{
  // Calculate error, voltage, and pwm
  float speed_error = right_motor_desired_speed - right_motor_vel;
  float voltage = Kp * speed_error;
  int pwm = float(voltage / read_battery_voltage());

  // Add pwm to current pwm
  right_motor_pwm += pwm;

  // Write motor speed and direction to motor controller
  if(right_motor_pwm > 0)
  {
    digitalWrite(Right_Motor_Direction,1);
    analogWrite(Right_Motor_Speed, min(255,abs(right_motor_pwm)));
  }
  else
  {
    digitalWrite(Right_Motor_Direction,0);
    analogWrite(Right_Motor_Speed, min(255,abs(right_motor_pwm)));
  }
}

float read_battery_voltage()
{
  // Return voltage on ADC 0 times a conversion factor
  return float(analogRead(0)) * 0.0138;
}

float calc_motor_vel(long curr_enc_pos, long prev_enc_pos)
{
  // Return mid point velocity estimation - (conversion factor) * (change in postion) * (time elapsed)
  return ((2 * 3.1415) / 3200) * (curr_enc_pos - prev_enc_pos) / (float(velocity_refresh_interval) / 1000);
}