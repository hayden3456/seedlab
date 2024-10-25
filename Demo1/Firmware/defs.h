/**
  * @file defs.h
  * @author Luca Ciancanelli
  *
  * @brief Header file for program defines
  */

#ifndef defs_h
#define defs_h

// Robot defines
#define WHEEL_RADIUS 7.5 // This is the radius of the wheel in centimeters
#define WHEEL_WIDTH 35 // This is the distance between the wheels in centimeters

// Software defines
// Main defines
#define PI 3.14159
#define FEET_TO_CM 30.48
#define DEG_TO_RAD 0.01745

// Scheduler variables
#define CONSOLE_REF 100
#define LOCALIZATION_UPDATE_PERIOD 40
#define CONTROL_UPDATE_PERIOD 40

// Communication defines
// Console parameters
#define BAUD_RATE 115200

#define MATLAB_COM true
#define MATLAB_TIME 15000

#define LEFT_MOTOR_PRINT true
#define RIGHT_MOTOR_PRINT true
#define POSITION_PRINT true
#define VELOCITY_PRINT true
#define PWM_PRINT false
#define DESIRED_POSITION_PRINT false

// Control defines
// Velocity controller
#define KP_VEL_LEFT 4
#define KP_VEL_RIGHT 4
#define OUTPUT_CAP_LEFT 127
#define OUTPUT_CAP_RIGHT 117

// Position controller
#define KP_POS_LEFT 0.65
#define KI_POS_LEFT 0.01
#define KP_POS_RIGHT 0.65
#define KI_POS_RIGHT 0.01

#define ANTI_WINDUP 0.25

// Movement controller
#define WAIT_TIME 2000

// Hardware defines
// Motor encoders
#define LEFT_MOTOR_INPUT_A 2
#define LEFT_MOTOR_INPUT_B 5
#define RIGHT_MOTOR_INPUT_A 3
#define RIGHT_MOTOR_INPUT_B 6

// Motor controller
#define MOTOR_ENABLE 4
#define LEFT_MOTOR_DIRECTION 7
#define RIGHT_MOTOR_DIRECTION 8
#define LEFT_MOTOR_SPEED 9
#define RIGHT_MOTOR_SPEED 10

// Thanks Arduino IDE
#define VEL_KP 0
#endif
