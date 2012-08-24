/*
* file: mov_interface.c
* brief:
* author: Yanling Jin, Amber Olsson
* date: 2011-05-03
* version: 0.1
* history	
*				   2011-05-09 Customized and integrated to the 
*											another movement algorithm (savas aydin)
*					 2011-12-09 Protocol done with its implementation so that
*											not used anymore
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "mov_interface.h"	


/* BitMasking example http://www.indiabix.com/technical/c/bits-and-bytes/ */
#define BIT_POS(N) ( 1U << (N) )
#define SET_FLAG(N, F) ( (N) |= (F) )
#define CLR_FLAG(N, F) ( (N) &= -(F) )


uint8_t to_MotorMessage(uint8_t ID0, uint8_t ID1, uint8_t increasing, uint8_t panicMode,
										 uint8_t right, uint8_t left, uint8_t front, uint8_t back)
{
 uint8_t motors = 0;

 if(ID0 == 1)
 SET_FLAG(motors, BIT_POS(7));
  
 if(ID1 == 1)
 SET_FLAG(motors, BIT_POS(6));

 if(increasing == 1)
 SET_FLAG(motors, BIT_POS(5));

 if(panicMode == 1)
 SET_FLAG(motors, BIT_POS(4));

 if(right == 1)
 SET_FLAG(motors, BIT_POS(3));

 if(left == 1)
 SET_FLAG(motors, BIT_POS(2));

 if(front == 1)
 SET_FLAG(motors, BIT_POS(1)) ;

 if(back == 1)
 SET_FLAG(motors, BIT_POS(0)) ;
  
 return motors;

}


void pWrite(uint8_t msg)
{
 #ifdef DEBUG
 printf("\nProtocol has this written to it: ");
 print_uint8_t_to_Binary(msg);
 #endif

}

void print_uint8_t_to_Binary(uint8_t bin)
{
 uint8_t temp,bit;
 int8_t counter;
 counter =sizeof(bin) * 8;

 for(counter = counter - 1; counter >= 0; counter--)
 {
 	temp = 1 << counter;
 	bit = temp & bin;
	if( bit == 0)
	{	
		printf("0");
	}
	else
	{
		printf("1");
	}
 }
 printf("\n");

}



void write_to_motor(unsigned char msg)
{
 // write_motor(msg);
}

/*
* write message to navigation
*/
void write_to_nav(void) 
{
 //write to navigation
}

/*
* read navigation Command
*/
void read_navCommand(void) 
{
 //read navigation command
}


/*
* read collision avoidance command
*/
void read_caCommand(void)
{
 //read collision avoidance command
    //PROTOCOL READ FROM CA

}
	

