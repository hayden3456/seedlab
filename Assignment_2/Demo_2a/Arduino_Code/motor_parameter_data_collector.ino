// File Name: motor_parameter_data_collection.ino
// Author: Luca Ciancanelli
// EENG350 - Section B
//
// Description:
// This Arduino script implements position and velocity
// tracking for each of the motors to acquire characterization
// data. Time, volatage, and velocity for each motor is printed
// to the console in a CSV format to allow easy collection using
// the MATLAB scripts.

// Function declarations
void scheduler();
void console_print(long current_time);
void left_motor_control(int dir, int pwm);
void right_motor_control(int dir, int pwm);
void left_motor_vel();
void right_motor_vel();
void left_motor_change();
void right_motor_change();

// Constants
// Encoder pin designators
const int Left_Motor_A_input = 2;
const int Right_Motor_A_input = 3;
const int Left_Motor_B_input = 5;
const int Right_Motor_B_input = 6;

// Motor pin designators
const int Motor_Enable = 4;
const int Left_Motor_Direction = 7;
const int Right_Motor_Direction = 8;
const int Left_Motor_Speed = 9;
const int Right_Motor_Speed = 10;

// Velocity refresh interval
const int velocity_refresh_interval_ms = 10;
const float velocity_refresh_interval_s = 0.01;

// Console refresh interval (ms)
const int console_refresh_interval = 100;

// Motor being run (0 - left motor, 1 - right motor)
const int motor_run = 1;

// Motor test pwm
const int motor_test_pwm = 96;

// Global variables
// Scheduler time tracker
long console_prev_time = 0;
long left_vel_prev_time = 0;
long right_vel_prev_time = 5;

// Encoder pin states
int left_motor_A_state = 0;
int right_motor_A_state = 0;
int left_motor_B_state = 0;
int right_motor_B_state = 0;

// Encoder rotation counter
long left_turn_count = 0;
long right_turn_count = 0;

// Motor states
int left_motor_pwm = 0;
int right_motor_pwm = 0;
int left_motor_dir = 0;
int right_motor_dir = 0;

// Velocity variables
float left_motor_vel_calc = 0;
float right_motor_vel_calc = 0;
long left_prev_pos = 0;
long right_prev_pos = 0;
int left_prev_vel_time = 0;
int right_prev_vel_time = 0;

void setup() {
  // Initialize COM port at 115200 baud
  Serial.begin(115200);

  // Set digital pins for motor encoders as inputs
  pinMode(Left_Motor_A_input, INPUT);
  pinMode(Right_Motor_A_input, INPUT);
  pinMode(Left_Motor_B_input, INPUT);
  pinMode(Right_Motor_B_input, INPUT);

  // Set motor control pins as outputs
  pinMode(Motor_Enable, OUTPUT);
  pinMode(Left_Motor_Speed, OUTPUT);
  pinMode(Right_Motor_Speed, OUTPUT);
  pinMode(Left_Motor_Direction, OUTPUT);
  pinMode(Right_Motor_Direction, OUTPUT);
  
  // Enable motor driver
  digitalWrite(Motor_Enable, HIGH);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(Left_Motor_A_input), left_motor_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Right_Motor_A_input), right_motor_change, CHANGE);

  // Start MATLAB data collection
  Serial.println("GO");
}

void loop() {
  // Run scheduler
  scheduler();
}

void scheduler()
{
  // Update current time
  long current_time = millis();

  // Print variables to console (if update period past)
  if(current_time >= (console_prev_time + console_refresh_interval))
  {
    console_prev_time = current_time;
    console_print(current_time);
  }

  // Update left motor velocity (if update period past)
  if(current_time > (left_vel_prev_time + velocity_refresh_interval_ms))
  {
    left_motor_vel();
    left_vel_prev_time = current_time;
  }

  // Update left motor velocity (if update period past)
  if(current_time > (right_vel_prev_time + velocity_refresh_interval_ms))
  {
    right_motor_vel();
    right_vel_prev_time = current_time;
  }

  // Start motor at 1s
  if(current_time == 1000)
  {
    if(motor_run == 0)
    {
      left_motor_control(1, motor_test_pwm);
    }
    else
    {
      right_motor_control(1, motor_test_pwm);
    }
  }
  
  // Stop motor at 4s
  if(current_time == 4000)
  {
    if(motor_run == 0)
    {
      left_motor_control(1, 0);
    }
    else
    {
      right_motor_control(1, 0);
    }

    // End MATLAB data collection
    Serial.println("STOP");
  }
}

void console_print(long current_time)
{
  // Print current runtime (s)
  Serial.print(float(console_prev_time) / 1000);

  // Check which motor is being simulated
  if(motor_run == 0)
  {
    // Print voltage applied to motor
    if(left_motor_pwm != 0)
    {
      Serial.print(",");
      Serial.print(float(analogRead(0)) * 0.0138 * (float(left_motor_pwm) / 255));
    }
    else
    {
      Serial.print(",");
      Serial.print(0);
    }

    // Print motor velocity
    Serial.print(",");
    Serial.println(left_motor_vel_calc);
  }
  else
  {
    // Print voltage applied to motor
    if(right_motor_pwm != 0)
    {
      Serial.print(",");
      Serial.print(float(analogRead(0)) * 0.0138 * (float(right_motor_pwm) / 255));
    }
    else
    {
      Serial.print(",");
      Serial.print(0);
    }

    // Print motor velocity
    Serial.print(",");
    Serial.println(right_motor_vel_calc);
  }
}

void left_motor_vel()
{
  // Calculate motor velocity (conversion factor) * (change in position) * (time elapsed)
  left_motor_vel_calc = float((((2*3.14159) / 3200) * (left_turn_count - left_prev_pos)) / velocity_refresh_interval_s);

  // Update previous turn count
  left_prev_pos = left_turn_count;
}
void right_motor_vel()
{
  // Calculate motor velocity (conversion factor) * (change in position) * (time elapsed)
  right_motor_vel_calc = float((((2*3.14159) / 3200) * (right_turn_count - right_prev_pos)) / velocity_refresh_interval_s);

  // Update previous turn count
  right_prev_pos = right_turn_count;
}

void left_motor_control(int dir, int pwm)
{
  // Update state variables
  left_motor_dir = dir;
  left_motor_pwm = pwm;

  // Write motor direction pin
  digitalWrite(Left_Motor_Direction, dir);

  // Write motor speed pin
  analogWrite(Left_Motor_Speed, pwm);
}

void right_motor_control(int dir, int pwm)
{
  // Update state variables
  right_motor_dir = dir;
  right_motor_pwm = pwm;

  // Write motor direction pin
  digitalWrite(Right_Motor_Direction, dir);

  // Write motor speed pin
  analogWrite(Right_Motor_Speed, pwm);
}

void left_motor_change()
{
  // Update state variables
  left_motor_A_state = digitalRead(Left_Motor_A_input);
  left_motor_B_state = digitalRead(Left_Motor_B_input);
  
  // Check pevious encoder state
  if(left_motor_A_state == left_motor_B_state)
  {
    // Increment turn counter
    left_turn_count += 2;
  }
  else
  {
    // Decrement turn counter
    left_turn_count -= 2;
  }
}

void right_motor_change()
{
  // Update state variables
  right_motor_A_state = digitalRead(Right_Motor_A_input);
  right_motor_B_state = digitalRead(Right_Motor_B_input);
  
  // Check pevious encoder state
  if(right_motor_A_state == right_motor_B_state)
  {
    // Decrement turn counter
    right_turn_count -= 2;
  }
  else
  {
    // Increment turn counter
    right_turn_count += 2;
  }
}