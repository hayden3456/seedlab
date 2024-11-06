/**
  * @file defs.h
  * @author Luca Ciancanelli, David Bowling and Tyler Sidlow
  *
  * @brief Header file for program defines
  */

#ifndef defs_h
#define defs_h

// Robot defines (cm)
#define WHEEL_RADIUS 7.5 
#define WHEEL_WIDTH 35

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
// I2C parameters
#define ARDUINO_ADDRESS 0x08
#define CMD_PACKET_SIZE 3

#define SEARCH_MODE 1
#define ANGLE_MODE 2
#define DISTANCE_MODE 3
#define STOP_MODE 4

// Console parameters
#define BAUD_RATE 115200

#define MATLAB_COM false
#define MATLAB_TIME 15000

#define LEFT_MOTOR_PRINT false
#define RIGHT_MOTOR_PRINT false
#define POSITION_PRINT false
#define VELOCITY_PRINT false
#define PWM_PRINT false
#define DESIRED_POSITION_PRINT false

// Control defines
// Operation mode
#define ROTATION_SPEED 1

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

// Blank value for previously deleted file (thanks Arduino IDE)
#define VEL_KP 0

#endif
