/*!
*   @file mov_interface.h
*   
*   @brief header file for mov_interface.c
*
*   @author Aydan Halilov
*						Savas Aydin
*
*		@reference TODO
*						
*   @date 2011-05-05
*
*   @history    2011-05-05 - Created header file for mov_main.c
*								2011-05-05 - changes according to coding standards
*								2011-05-09 - made compatible with other modules
*								2011-05-15 - protocol functions added
* 							2011-05-16 - added simulator.c variables and functions
*
*/

#ifndef MOV_INTERFACE_H
#define MOV_INTERFACE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

/* desired attitude */
typedef struct attitude
{
 int16_t roll;
 int16_t pitch;
 int16_t yaw;
}Attitude;

/* struct coming from navigation */
typedef struct mov_navigation
{
 int16_t type;   /* 0=auto or 1=manual */
 int16_t order;  /* hover=0 moving=1 landing=2 lift_off=3 right=4 left=5 back=6*/
 int16_t height;   /* target height */
 int16_t distance;
 int16_t yaw;   /* to be used in auto mode - target yaw */
}MovementCommand;

/* struct coming from motor */
typedef struct motor_throttle
{
 int16_t actual_front_throttle;
 int16_t actual_back_throttle;
 int16_t actual_left_throttle;
 int16_t actual_right_throttle;
}Motor_throttle;

/*
 * Declaration of Global Variables
 */
 
/* Global variable to calculate actual total throttle */
int16_t actual_total_throttle;


/* mov_main.c */
int16_t mov_run();
int16_t mov_init();
int16_t mov_setup_auto(int const roll, int const pitch, int const yaw);
int16_t mov_setup_manual(int roll, int pitch, MovementCommand *navigation_data, 
  									 Motor_throttle *motor_throttle);


/* mov_movement.c */
#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000

int8_t mov_control_motor(Attitude *control_attitude, int16_t target_altitude, 
											 Motor_throttle *motor_throttle);


/* mov_sensor.c */
float sonar_distance(int sonarPin);
void printIt();
#define SONAR_PIN 5 /*TODO confirm sonar pin */


/* simulator.c */
MovementCommand* read_command(FILE *file);
int get_loc(char line[], char c, int indexOfChar);
//int16_t do_sensor_simulation(int16_t currentSensorValue, uint8_t command);
#define COLUMN 5


/* mov_motor_message.c */
uint8_t to_MotorMessage(uint8_t ID0, uint8_t ID1, uint8_t increasing, uint8_t panicMode,
										 uint8_t right, uint8_t left, uint8_t front, uint8_t back);
void pWrite(uint8_t msg);
void print_uint8_t_to_Binary(uint8_t bin);


/* mov_attiude.c */
Attitude* mov_attitude_control(Attitude *desired_attitude);
int16_t mov_limit(int16_t value, int16_t max, int16_t min);
int16_t mov_normalize(int angle);
int16_t mov_altitude_control(int16_t target_altitude, Motor_throttle *motor_throttle);

#define DT 0.06 // the time diff. between every calling 
/* TODO - stab defined dt as 0.06 */


/* TODO Defined macro values will be accurated in test */
/* PI constant of attitude control */
#define KP_ROLL 4.0
#define KI_ROLL 0.15
#define STABLE_KP_RATE_ROLL 1.2
#define MAX_ROLL_OUTPUT 250
#define MIN_ROLL_OUTPUT -250
#define KP_PITCH 4.0
#define KI_PITCH 0.15
#define STABLE_KP_RATE_PITCH 1.2
#define MAX_PITCH_OUTPUT 250
#define MIN_PITCH_OUTPUT -250
#define KP_YAW 3.0
#define KI_YAW 0.15
#define STABLE_KP_RATE_YAW 2.4
#define MAX_YAW_OUTPUT 250
#define MIN_YAW_OUTPUT -250

/* to limit the euler angles and their rates */
#define MAX_ROLL_ANGLE 25
#define MAX_I_ROLL_RATE 20
#define MAX_PITCH_ANGLE 25
#define MAX_I_PITCH_RATE 20
#define MAX_YAW_ANGLE 60
#define MAX_I_YAW_RATE 20


/* PID constant of altitude control */
#define KP_ALTITUDE 0.8
#define KI_ALTITUDE 0.3
#define KD_ALTITUDE 0.7

/* to limit the altitude error and its rate */
#define MAX_ERROR_ALTITUDE 60
#define MAX_I_ERROR_ALTITUDE 150

/* to be used to movement control under mov_main.c function*/
#define TURN_LEFT 20
#define TURN_RIGHT -20
#define FORWARD -50
#define BACKWARD 20

#define DESIRED_PITCH 0
#define DESIRED_ROLL 0

/*to test flag*/

#define FILTER_ROLL 0
#define FILTER_PITCH 0
#define FILTER_YAW 0
#define SENS_ROLL 5
#define SENS_PITCH 5
#define SENS_YAW 5


#endif

