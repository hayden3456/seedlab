// File Name: motor_parameter_data_collection.ino
// Authors: Luca Ciancanelli and David Bowling
// EENG350 - Section B
//
// Description:
// This Arduino script implements position and velocity
// tracking for each of the motors to acquire characterization
// data. Time, volatage, and velocity for each motor is printed
// to the console in a CSV format to allow easy collection using
// the MATLAB scripts.

// Function declarations
void scheduler(); // Orders when everything happens
void console_print(long current_time); // Prints out whatever we need recorded
// controls the commands to the respective motors
void left_motor_control(int dir, int pwm);
void right_motor_control(int dir, int pwm);
// controls the motor positions
void left_motor_pos();
void right_motor_pos();
// controls the motors velocities
void left_motor_vel();
void right_motor_vel();
// Reads the changes in the motor encoders
void left_motor_change();
void right_motor_change();

// Block Diagram functions
long subtractor(long left_in, long bottom_in);

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

// This is the input pin from the Rasberry Pi
const int rasberryPiInput = 3;

// Velocity refresh interval
const int velocity_refresh_interval_ms = 10;
const float velocity_refresh_interval_s = 0.01;

// Console refresh interval (ms)
const int console_refresh_interval = 100;

// Motor being run (0 - left motor, 1 - right motor)
const int motor_run = 1;

// Motor test pwm
const int motor_test_pwm = 96;

// Desired positions
long desired_pos_left_motor = 0, desired_pos_right_motor = 0;

// integral error for position controller
float left_vel_error = 0, right_vel_error = 0; // Values for velocity error
float left_pos_error = 0, right_pos_error = 0; // Values for position error

// Global variables
// Scheduler time tracker
long console_prev_time = 0;
long left_vel_prev_time = 0;
long right_vel_prev_time = 5;

// Encoder pin states
int left_motor_A_state = 0, right_motor_A_state = 0, left_motor_B_state = 0, right_motor_B_state = 0;

// Encoder rotation counter
long measured_pos_left_motor = 0, measured_pos_right_motor = 0;

// Motor states
int left_motor_pwm = 0, right_motor_pwm = 0, left_motor_dir = 0, right_motor_dir = 0;

// Velocity variables
float left_motor_vel_calc = 0, right_motor_vel_calc = 0;
long left_prev_pos = 0, right_prev_pos = 0;
int left_prev_vel_time = 0, right_prev_vel_time = 0;

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
  }
  right_motor_pos_controller();
  left_motor_pos_controller();
  // End MATLAB data collection
  Serial.println("STOP");
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

// This is the given code for the PID controller portion
void pwm_controller(){
  for(i=0;i<2;i++) {
  pos_error[i] = desired_pos[i] - actual_pos[i];
  integral_error[i] = integral_error[i] + pos_error[i]*((float)desired_Ts_ms /1000);
  desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
  error[i] = desired_speed[i] - actual_speed[i];
  Voltage[i] = Kp*error[i];
  }
}

// This is the function to take in the signal from the Rasberry Pi and set a desired position
void find_desired_pos(){
  long read_pos[2] = {0};
  read_pos =  analogRead(rasberryPiInput);
  left_motor_desired_pos = read_pos(0);
  right_motor_desired_pos = read_pos(1);
}

// This is the funciton that will take in the given desired position and find a desired velocity for the motor
void motor_PID_controller(long desired_pos, long measured_pos){
  long pos_error  = subtractor(desired_pos, measured_pos);
  long integral_error = integral_error + pos_error*((float)desired_Ts_ms /1000);
  desired_velocity = Kp_pos * pos_error + Ki_pos * integral_error;
}

// This is the function that will take the desired velocity and set the voltage for the motors to meet that velocity
void motor_Velocity_controller(long desired_velocity, long measured_velocity){
  long vel_error = desired_velocity - measured_velocity;
  motor_voltage = Kp*vel_error;
  if(motor_voltage < 0){
    motor_dir = 0
    motor_voltage = -motor_voltage; // set it positive
  }
  else{motor_dir = 1;}
}

// This is the uncompleted position controller
void left_motor_pos_controller(){
  pos_error subtractor(long left_in, long bottom_in);
  
}

void left_motor_vel()
{
  // Calculate motor velocity (conversion factor) * (change in position) * (time elapsed)
  left_motor_vel_calc = float((((2*3.14159) / 3200) * (measured_pos_left_motor - left_prev_pos)) / velocity_refresh_interval_s);

  // Update previous turn count
  left_prev_pos = measured_pos_right_motor;
}
void right_motor_vel()
{
  // Calculate motor velocity (conversion factor) * (change in position) * (time elapsed)
  right_motor_vel_calc = float((((2*3.14159) / 3200) * (measured_pos_right_motor - right_prev_pos)) / velocity_refresh_interval_s);

  // Update previous turn count
  right_prev_pos = measured_pos_right_motor;
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
    measured_pos_left_motor += 2;
  }
  else
  {
    // Decrement turn counter
    measured_pos_left_motor -= 2;
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
    measured_pos_right_motor -= 2;
  }
  else
  {
    // Increment turn counter
    measured_pos_right_motor += 2;
  }
}

long subtractor(long left_in, long bottom_in){
  return left_in - bottom_in;
}
