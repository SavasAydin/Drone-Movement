/*!
*   @file mov_main.c
*   
*   @brief main file of mov module
*
*   @author 
*						Savas Aydin Aydan Halilov
*
*		@reference TODO
*						
*   @date 2011-05-05
*
*   @history    2011-05-05 - initial version
*								2011-05-07 - changes according to coding standards
*								2011-05-09 - made compatible with other modules
*								2011-05-10 - debug and final review before implementation 
*								2011-06-10 - updated due to design changes
*								2011-05-12 - added simulation for navigation group to test 
*														 the code on mega
*								2011-05-15 - more organized for integration
*								2011-05-16 - small changes for simulation
*								2011-05-19 - test flag and main function added to run the code 
*/

/* 
 * auto intended to move front automatically with a stable angle 
 * until meet with obstacle then move back with the same angle.
 * manual intended to move the quadro by commands from remote controller
 */

#ifdef PC
#include <stdio.h>
#include <stdlib.h>
/* PC headers */

#elif ARDUINO
#include "WProgram.h"
#include "proto_lib.h"
#endif /* PC ARDUINO */

#include "mov_interface.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>


int16_t mov_setup_manual(int roll, int pitch, MovementCommand *nav_to_mov,
											Motor_throttle *motor_throttle)
{
 #ifdef DEBUG
 #ifdef PC
 printf("inside setup.. \n");
 #elif defined ARDUINO
 Serial.println("inside setup.. \n");
 #endif
 #endif

 Attitude *desired_attitude, *control_attitude;
 desired_attitude = (Attitude*)malloc(sizeof(Attitude));
 
 int16_t target_altitude;
 desired_attitude->roll = roll;
 desired_attitude->pitch = pitch;
 desired_attitude->yaw = nav_to_mov->yaw;
 
 target_altitude = nav_to_mov->height; 

 control_attitude = mov_attitude_control(desired_attitude);
 mov_control_motor(control_attitude, target_altitude, motor_throttle);

 return 0;
 
}

int16_t mov_setup_auto(int const roll, int const pitch, int const yaw)
{
	/* TODO */
	
	return 0;
	
}

int16_t mov_init()
{
 MovementCommand *nav_to_mov; 
// nav_to_mov = (MovementCommand*)malloc(sizeof(MovementCommand));
 /*
 int i;
 for(i=0;i<ROW;i++)
 {
  *nav_to_mov = (MovementCommand*)malloc(sizeof(MovementCommand)*ROW);
 }
 */
 
 /* to be used for simulation */
 #ifdef SIMULATOR
 FILE *file;
 file = fopen("input.txt", "r");
 nav_to_mov = read_command(file);
 #elif ARDUINO
 nav_to_mov = proto_read_move_to_nav();
 #endif
 
 #ifdef DEBUG
 #ifdef PC
 printf("inside mov_init.. \n");
 #elif defined ARDUINO
 Serial.println("inside mov_init.. \n");
 #endif
 #endif
 
 Motor_throttle *motor_throttle;
 motor_throttle = (Motor_throttle*)malloc(sizeof(Motor_throttle));

/* TODO read throttle values from motor group */
 #ifdef ARDUINO
 motor_throttle = proto_read_throttle();
 #elif TEST
 motor_throttle->actual_front_throttle = 1300;
 motor_throttle->actual_back_throttle = 1300;
 motor_throttle->actual_left_throttle = 1300;
 motor_throttle->actual_right_throttle = 1300;
 printf("total throttle is = %d\n", motor_throttle->actual_front_throttle +
 																		motor_throttle->actual_back_throttle +
																		motor_throttle->actual_left_throttle +
																		motor_throttle->actual_right_throttle);
 #endif
 if(nav_to_mov->type == 0)
 {
  /* TODO - auto mode */
 }
 else
 {
 
  /* hover=0 =1 landing=2 lift_off=3 right=4 left=5 back=6 */
 
  /* landing */
  if(nav_to_mov->order == 2) 
  {  
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries landing.. \n");
   #elif defined ARDUINO
   Serial.println("drone tries landing.. \n");
   #endif
   #endif
 
  nav_to_mov->yaw = 0;  /* target yaw is 0 */
  nav_to_mov->height = 0; /* target height will be 0 */
  mov_setup_manual(DESIRED_ROLL, DESIRED_PITCH, nav_to_mov, motor_throttle);
  }
 
  /* lift_off */
  else if(nav_to_mov->order == 3)
  {
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries lifting... \n");
   #elif defined ARDUINO
   Serial.println("drone tries lifting \n");
   #endif
   #endif
 
   nav_to_mov->yaw = 0;
   mov_setup_manual(DESIRED_ROLL, DESIRED_PITCH, nav_to_mov, motor_throttle);
  }
 
  /* hovering */
  else if(nav_to_mov->order == 0)
  {
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries hovering.. \n");
   #elif defined ARDUINO
   Serial.println("drone tries hovering.. \n");
   #endif
   #endif
   
   nav_to_mov->yaw = 0;
   mov_setup_manual(DESIRED_ROLL, DESIRED_PITCH, nav_to_mov, motor_throttle);
  }
 
  /* forward aka moving */
  else if(nav_to_mov->order == 1)
  {
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries moving... \n");
   #elif defined ARDUINO
   Serial.println("drone tries moving.. \n");
   #endif
   #endif
  
   nav_to_mov->yaw = 0;
   mov_setup_manual(FORWARD, DESIRED_PITCH, nav_to_mov, motor_throttle);
  }
 
  /* back */
  else if(nav_to_mov->order == 6)
  {
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries going_back \n");
   #elif defined ARDUINO
   Serial.println("drone tries going_back.. \n");
   #endif
   #endif 

   nav_to_mov->yaw = 0;
   mov_setup_manual(BACKWARD, DESIRED_PITCH, nav_to_mov, motor_throttle);
  } 

  /* TODO turn right or left depends on the direction of turning or propeller. */
  /* right */
  else if(nav_to_mov->order == 4)
  {
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries turning_right.. \n");
   #elif defined ARDUINO
   Serial.println("drone tries turning_right.. \n");
   #endif
   #endif 

   nav_to_mov->yaw = TURN_RIGHT;
   mov_setup_manual(DESIRED_ROLL, DESIRED_PITCH, nav_to_mov, motor_throttle);
  }
 
  /* left */
  else if(nav_to_mov->order == 5)
  {
   #ifdef DEBUG
   #ifdef PC
   printf("drone tries turning_left.. \n");
   #elif defined ARDUINO
   Serial.println("drone tries turning_left\n");
   #endif
   #endif
   
   nav_to_mov->yaw = TURN_LEFT;
   mov_setup_manual(DESIRED_ROLL, DESIRED_PITCH, nav_to_mov, motor_throttle);
  }
 } 
 free(nav_to_mov);
 return 0;
 
}

int16_t mov_run(void)
{
 #ifdef DEBUG
 #ifdef PC
 printf("inside mov_run.. \n");
 #elif defined ARDUINO
 Serial.println("inside mov_run.. \n");
 #endif
 #endif
 
 mov_init();
 return 0;

}

/* to test it indepently */

int main(void)
{
	mov_run();
	return 0;
}




