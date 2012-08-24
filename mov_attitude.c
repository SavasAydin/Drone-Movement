/*!
*   @file mov_attitude.c
*   
*   @brief attitude control
*
*   @author Savas Aydin Aydan Halilov
*						
*
*		@reference necessary calculations and idea of how drone should move
*
*							 www.ArduCopter.com - www.DIYDrones.com
*							 An Open Source Arduino based multicopter.
*					 		 Version  : v1.0, 15 December 2010
*							 Author(s): ArduCopter Team  
*												  Ted Carancho (AeroQuad), Jose Julio, Jordi Muñoz,
*													Jani Hirvinen, Ken McEwans, Roberto Navoni, 
*													Randy Mackay, Sandro Benigno, Chris Anderson
*						
*   @date 2011-05-03
*
*   @history    2011-05-03 - initial prototype
*								2011-05-05 - changes according to coding standards
*								2011-05-09 - made compatible with other modules
*								2011-05-15 - added protocol functions
*								2011-05-16 - changes done to simulate navigation
*
*/

#ifdef PC
#include <stdio.h>
/* PC headers */

#elif ARDUINO
#include "WProgram.h"
#include "proto_lib.h"
#endif /* PC ARDUINO */

#include "mov_interface.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

/* Global variables for attitude_control function */
 /* to be used for accurate attitude values */
 float stable_roll,stable_pitch, stable_yaw; 
 /* differences between desired and actual values */
 float error_roll=0;
 float error_pitch=0;
 float error_yaw=0;  
 /* to store integral component of PID */
 float roll_I, pitch_I, yaw_I; 

/* Global variables for mov_altitude_control function */
 int error_altitude_old;
 int error_altitude;
 float altitude_I, altitude_D, command_altitude;

  
Attitude* mov_attitude_control(Attitude *desired_attitude)
{ 
 Attitude *control_attitude; /*return struct of attitude control */
 control_attitude = (Attitude*)malloc(sizeof(Attitude));
 

 Attitude *sens, *filter;
 sens = (Attitude*)malloc(sizeof(Attitude));
 filter = (Attitude*)malloc(sizeof(Attitude));
 
 /*ROLL CONTROL */
 /* TODO sens->roll from stab */
 #ifdef ARDUINO
 filter = proto_stabReadAttitude;
 sens = proto_stabReadGyro;
 #elif TEST
 sens->roll = SENS_ROLL;
 #endif
 error_roll = desired_attitude->roll - sens->roll; 
 /* will be changed after testing */
 error_roll = mov_limit(error_roll, MAX_ROLL_ANGLE, -MAX_ROLL_ANGLE); 
 
 roll_I += error_roll*DT;
 /* will be changed after testing */
 roll_I  = mov_limit(roll_I, MAX_I_ROLL_RATE, -MAX_I_ROLL_RATE); 

 /* PID roll angle controll 
  * Aim is to minimize error rate
  */
 stable_roll = KP_ROLL*error_roll + KI_ROLL*roll_I;
 
 /* PD roll rate controll */
 /* roll angle after drift correction */
 #ifdef TEST
 filter->roll = FILTER_ROLL;
 #endif
 error_roll = stable_roll - filter->roll;  /* TODO filter->roll*/
 control_attitude->roll = STABLE_KP_RATE_ROLL*error_roll;
 control_attitude->roll = mov_limit(control_attitude->roll, 
 																MAX_ROLL_OUTPUT, MIN_ROLL_OUTPUT);

 /*PITCH CONTROL */
 #ifdef TEST
 sens->pitch = SENS_PITCH;
 #endif
 error_pitch = desired_attitude->pitch - sens->pitch; 
 /* will be changed after testing */
 error_pitch = mov_limit(error_pitch, MAX_PITCH_ANGLE, -MAX_PITCH_ANGLE); 
 
 pitch_I += error_pitch*DT;
 /* will be changed after testing */
 pitch_I  = mov_limit(pitch_I, MAX_I_PITCH_RATE, -MAX_I_PITCH_RATE); 
 
 /* PID picth angle control  */
 stable_pitch = KP_PITCH * error_pitch + KI_PITCH * pitch_I;

 /* PD pitch rate control */
 /* pitch angle after drift correction */
 #ifdef TEST
 filter->pitch = FILTER_PITCH;
 #endif
 error_pitch = stable_pitch - filter->pitch;
 control_attitude->pitch = STABLE_KP_RATE_PITCH * error_pitch;
 control_attitude->pitch = mov_limit(control_attitude->pitch, 
 																 MAX_PITCH_OUTPUT, MIN_PITCH_OUTPUT);

 /* YAW CONTROL */
 #ifdef TEST
 sens->yaw = SENS_YAW;
 #endif
 error_yaw = desired_attitude->yaw - sens->yaw; 
 error_yaw = mov_normalize(error_yaw);
 /* will be changed after testing */
 error_yaw = mov_limit(error_yaw, MAX_YAW_ANGLE, -MAX_YAW_ANGLE); 

 yaw_I += error_yaw * DT;
 yaw_I  = mov_limit(yaw_I, MAX_I_YAW_RATE, -MAX_I_YAW_RATE);

 /* PID yaw angle control */
 stable_yaw = KP_YAW * error_yaw + KI_YAW * yaw_I;
 /* PD yaw rate control */
 /* yaw angle after drift correction */
 #ifdef TEST
 filter->yaw = FILTER_YAW;
 #endif
 error_yaw = stable_yaw - filter->yaw;  
 control_attitude->yaw = STABLE_KP_RATE_YAW * error_yaw;
 control_attitude->yaw = mov_limit(control_attitude->yaw, 
 															 MAX_YAW_OUTPUT, MIN_YAW_OUTPUT);

 return control_attitude;
}

/* controls the quadro for desired altitude */
int16_t mov_altitude_control(int16_t target_altitude, Motor_throttle *motor_throttle)
{			  			
 actual_total_throttle = motor_throttle->actual_front_throttle + 
 												 motor_throttle->actual_back_throttle +
 				  							 motor_throttle->actual_right_throttle + 
 				  							 motor_throttle->actual_left_throttle; 	
				 
 error_altitude_old = error_altitude;
 /*TODO PIN for sonar will be determined */
 #ifdef TEST
 float son_dist = 150.0;
 error_altitude = target_altitude - son_dist;
 #elif ARDUINO
 error_altitude = target_altitude - sonar_distance(SONAR_PIN);
 #endif
 /* might change for desired altitude mov_limits */
 error_altitude = mov_limit(error_altitude, 
 														MAX_ERROR_ALTITUDE, -MAX_ERROR_ALTITUDE); 
 
 altitude_D = (float)(error_altitude-error_altitude_old)/DT;
 altitude_I = (float)(error_altitude*DT);
 /* might change for desired mov_limits */
 altitude_I = mov_limit(altitude_I, 
 												MAX_I_ERROR_ALTITUDE,-MAX_I_ERROR_ALTITUDE); 
 
 /*TODO get initial_throttle */
 command_altitude = actual_total_throttle + KP_ALTITUDE*error_altitude + 
 										KD_ALTITUDE*altitude_D + KI_ALTITUDE*altitude_I;

 return command_altitude;			
 
}

/* limits the value in the max and min range */
int16_t mov_limit(int16_t value, int16_t max, int16_t min)
{
 if(value >= max)
 {
 	return max;
 }
 else if (value <= min)
 {
	return min;
 }
 else
 {
 	return value;
 }

}

/* normalizes angle between -180 and 180 */
int16_t mov_normalize(int angle)
{
 if(angle > 180)
 {
  return angle -= 360;
 }
 else if(angle < -180)
 {
  return angle += 360;
 }
 return 0; 
}

