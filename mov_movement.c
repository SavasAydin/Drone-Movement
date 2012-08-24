/*!
*   @file mov_movement.c
*   
*   @brief send accurate data to motor 
*
*   @author 
*						Savas Aydin
*
*		@reference TODO
*						
*   @date 2011-05-04
*
*   @history    2011-05-04 - initial version
*								2011-05-05 - changes according to coding standards
*								2011-05-09 - made compatible with other modules
*								2011-05-10 - added message passing between movement & motor +
*														 debug and final review before implementation
*								2011-05-15 - more organized for integration
*								2011-05-16 - small changes for simulation
*/

#ifdef PC
#include <stdio.h>
#elif ARDUINO
#include "WProgram.h"
#include "proto_lib.h"
#endif /* PC ARDUINO */

#include "mov_interface.h"

#include <stdint.h>
#include <stdio.h>
/* 
 * initiliaze and compare desired throttles and actual throttles 
 * for each motor then send correct message to motor module
 */
int8_t mov_control_motor(Attitude *control_attitude, int16_t target_altitude, 
											 Motor_throttle *motor_throttle)
{
 int8_t message;
 
 int16_t desired_right_throttle, desired_left_throttle; 
 int16_t desired_front_throttle, desired_back_throttle;
 int16_t desired_altitude_throttle;
 
 	
 /* 
  * the calculation for desired_xxx_throttle is taken from the same 
  * project that is mentioned on mov_attitude.c
  * (see the reference in the file mov_attitude.c
  */
 desired_front_throttle = mov_limit(motor_throttle->actual_front_throttle + 
 																control_attitude->roll - 
 																control_attitude->yaw, 
 																MAX_THROTTLE, MIN_THROTTLE);		
 #ifdef TEST
 printf("desired front throttle = %d\n",desired_front_throttle);	
 #endif
 
 desired_right_throttle = mov_limit(motor_throttle->actual_right_throttle - 
 																control_attitude->pitch + 
 																control_attitude->yaw, 
							  								MAX_THROTTLE, MIN_THROTTLE);
 #ifdef TEST
 printf("desired right throttle = %d\n",desired_right_throttle);
 #endif
 
 desired_left_throttle = mov_limit(motor_throttle->actual_left_throttle + 
 															 control_attitude->pitch + 
 															 control_attitude->yaw, 
															 MAX_THROTTLE, MIN_THROTTLE);
 #ifdef TEST
 printf("desired left throttle = %d\n",desired_left_throttle); 		
 #endif
 
 desired_back_throttle = mov_limit(motor_throttle->actual_back_throttle - 
 															 control_attitude->roll - 
 															 control_attitude->yaw, 
 															 MAX_THROTTLE, MIN_THROTTLE);
 #ifdef TEST
 printf("desired back throttle = %d\n",desired_back_throttle);  															 
#endif
 
 actual_total_throttle = motor_throttle->actual_front_throttle + 
 												 motor_throttle->actual_back_throttle +
 				  							 motor_throttle->actual_right_throttle + 
 				  							 motor_throttle->actual_left_throttle; 	

 desired_altitude_throttle = mov_limit(mov_altitude_control(target_altitude, 
 																												motor_throttle), 
 																	 		 4*MAX_THROTTLE, 4*MIN_THROTTLE);
 #ifdef TEST
 printf("desired total throttle = %d\n",desired_altitude_throttle);  		
 #endif
 
 /* 
  * Compares desired and actual throttle for each motor
  * and send necessary messages to each motor 
  */  																	 
 /* Controls front motor throttle */	
 if(desired_front_throttle > motor_throttle->actual_front_throttle)
 {
 	/* increase front motor -- 10 10 00 10 */
  printf(" front motor increasing..\n");
 	message = 0xA2;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" front motor increasing..\n");
  #elif defined ARDUINO
  Serial.println(" front motor increasing..\n");
  #endif
  #endif	
 }
 
 else if(desired_front_throttle < motor_throttle->actual_front_throttle)
 {
 	/* decrease front motor -- 10 00 00 10 */
 	message = 0x82;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" front motor decreasing..\n");
  #elif defined ARDUINO
  Serial.println(" front motor decreasing..\n");
  #endif
  #endif
 }
 
 else 
 {
 	/* do nothing */
  #ifdef DEBUG
  #ifdef TEST
  #elif defined ARDUINO
  Serial.println(" no changes\n");
  #endif
  #endif
 }

 /* Controls back motor throttle */ 	
 if(desired_back_throttle > motor_throttle->actual_back_throttle)
 {
 	/* increase back motor -- 10 10 00 01 */
 	message = 0xA1;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" back motor increasing..\n"); 	
  #elif defined ARDUINO
  Serial.println(" back motor increasing..\n"); 	
  #endif
  #endif
 }
 
 else if(desired_back_throttle < motor_throttle->actual_back_throttle)
 {
 	/* decrease back motor -- 10 00 00 01 */
 	message = 0x81;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST 	
  printf(" back motor decreasing..\n"); 	
  #elif defined ARDUINO
  Serial.println(" back motor decreasing..\n"); 	
  #endif
  #endif 
 }

 else 
 {
 	/* do nothing */
  #ifdef DEBUG
  #ifdef PC
  #elif defined ARDUINO
  Serial.println(" no changes\n");
  #endif
  #endif
 }
 	
 /* Controls left motor throttle */	
 if(desired_left_throttle > motor_throttle->actual_left_throttle)
 {
 	/* increase left motor -- 10 10 01 00 */
 	message = 0xA4;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" left motor increasing..\n"); 	
  #elif defined ARDUINO
  Serial.println(" left motor increasing..\n"); 	
  #endif
  #endif 
 }
 else if(desired_left_throttle < motor_throttle->actual_left_throttle)
 {
 	/* decrease left motor - 10 00 01 00 */
 	message = 0x84;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" left motor decreasing..\n"); 	
  #elif defined ARDUINO
  Serial.println(" left motor decreasing..\n"); 	
  #endif
  #endif 
 }
  
 else 
 {
 	/* do nothing */
  #ifdef DEBUG
  #ifdef TEST
  #elif defined ARDUINO
  Serial.println(" no changes\n");
  #endif
  #endif
 }
 	
 /* control right motor throttle */	
 if(desired_right_throttle > motor_throttle->actual_right_throttle)
 {
 	/* increase right motor -- 10 10 10 00 */
	message = 0xA8; 	
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" right motor increasing..\n"); 	
  #elif defined ARDUINO
  Serial.println(" right motor increasing..\n"); 	
  #endif
  #endif 
 }
 
 else if(desired_right_throttle < motor_throttle->actual_right_throttle)
 {
 	/* decrease right motor -- 10 00 10 00 */
 	message = 0x88;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" right motor decreasing..\n"); 	
  #elif defined ARDUINO
  Serial.println(" right motor decreasing..\n"); 	
  #endif
  #endif 
 }

 else 
 {
 	/* do nothing */
  #ifdef DEBUG
  #ifdef TEST
  printf(" no changes..\n");
  #elif defined ARDUINO
  Serial.println(" no changes\n");
  #endif
  #endif
 }

 if(desired_altitude_throttle > actual_total_throttle)
 {
 	/* increase all the motors -- 10 10 11 11 */
 	message = 0xAF;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif 	
  #ifdef DEBUG
  #ifdef TEST
  printf(" all motors increasing..\n");
  #elif defined ARDUINO
  Serial.println(" all motors increasing\n");
  #endif
  #endif
 }
 else if(desired_altitude_throttle < actual_total_throttle)
 {
 	/* decrease all the motors -- 10 00 11 11 */
 	message = 0x8F;
 	#ifdef ARDUINO
 	proto_write_motor(message);
 	#endif
  #ifdef DEBUG
  #ifdef TEST
  printf(" all motors decreasing..\n");
  #elif defined ARDUINO
  Serial.println(" all motors decreasing..\n");
  #endif
  #endif
 }  
 else 
 {
  printf(" no changes..\n"); 
 	/* do nothing */
  #ifdef DEBUG
  #ifdef TEST
  printf(" no changes..\n");
  #elif defined ARDUINO
  Serial.println(" no changes\n");
  #endif
  #endif
 }
 return 0;
}

