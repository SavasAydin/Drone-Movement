/*
 * file:         mov_simulator.c
 * brief:
 * author:       Yanling Jin, Amber Olsson
 * date:         2011-05-03
 * version:      0.1
 * history       2011-05-12 Customized and integrated to the 
 *													another algorith of movement (savas aydin)
 *							 2011-05-16 - small changes for better simulation (savas aydin)
 *							 
 * detail:
 */


#include "mov_interface.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>


/*
 * 
 */
MovementCommand* read_command(FILE *file)
{

 MovementCommand *p; 
 p = (MovementCommand*)malloc(sizeof(MovementCommand));

 char line[60];
 if (fgets(line, sizeof(line) + 1, file) != NULL) 
 {
	printf("$$$$$$$$$$$$$$LINE: %s\n", line);
	int i;
	for (i = 1; i < 6	; i++) 
	{
	 int start = get_loc(line, ':', i);
	 int end = get_loc(line, ' ', i);
	 int length = end - start;
	 char *temp = (char *) malloc(length-1*sizeof(char));
	 strncpy(temp, line + start, length-1);
	 if(i == 1)
	 {
	  p->type = atoi(temp);
		printf("Type is %d\n", p->type); 
   }
   else if(i == 2)
   {
		p->order = atoi(temp);
  	printf("Order is %d\n", p->order);   
   }
   else if(i == 3)
   {
    p->height = atoi(temp);
		printf("Height is %d\n", p->height);   
   }
   else if( i == 4)
   {
		p->distance = atoi(temp);
		printf("Distance is %d\n", p->distance);   
   }
   else if(i == 5)
   {
   		p->yaw = atoi(temp);
			printf("Yaw is %d\n", p->yaw);
	 }
   else
   {
		printf("Don't give me invalid value\n");   
   }
	}	

 }
 else 
 {
	return ;
 }
 return p;
}

/*
 * 
 */
 /*
void assignValue(int index, char *temp)
{
 switch (index) 
 {
    case 1:
			p->type = atoi(temp);
			printf("Type is %c\n", p->type); 
			break;
    case 2:
			p->order = atoi(temp);
			printf("Order is %c\n", p->order);
			break;
    case 3:
			p->height = atoi(temp);
			printf("Height is %d\n", p->height);
			break;
    case 4:
			p->distance = atoi(temp);
			printf("Distance is %d\n", p->distance);
			break;
    case 5:
			p->yaw = atoi(temp);
			printf("Yaw is %d\n", p->yaw);
			break;
    default:
			printf("Don't give me invalid value\n");
			break;
 }
}
*/
/*
 * 
 */
int get_loc(char line[], char c, int indexOfChar)
{
 int i = 0;
 int count = 0;
 while (count != indexOfChar) 
 {
	while (line[i] != c) 
	{
	 i++;
	}
	count++;
	i = i + 1;
 }
 return i;
 
}

/*
* (rand() % (max - min + 1) + min)
* for simulating the sensor data
*
* Only changing values if simulation is on
*/
int16_t do_sensor_simulation(int16_t currentSensorValue, uint8_t command)
{
 int8_t i = (rand() % (6 - 0 + 1) + 0);

 int16_t new;

 switch (command) 
 {
    case 1:
			new = currentSensorValue + i;
			break;
 		case 2:
			new = currentSensorValue - i;
			break;
 }
 return new;
 
}


/*
int main(void)
{
 file = fopen("input.txt", "r");
 read_command();
 if (read_command()== 0) 
 {
  fclose(file);
 }
 return 0;
}
*/
