#ifndef FIRSTORDERFILTER_H_
#define FIRSTORDERFILTER_H_

#include <stdint.h>

#define Number_Of_FirstOrderFilter	15
#define Current_1		0
#define Current_2		1
#define Current_3		2
#define Current_4		3
#define	Magneto_X		4
#define Magneto_Y		5
#define Magneto_Z		6
#define SRF05_Sensor	7
#define Accel_X			8
#define Accel_Y			9
#define Accel_Z			10
#define Gyro_X			11
#define Gyro_Y			12
#define Gyro_Z			13
#define AccelZ_HP		14

typedef struct firstOrderFilterData {
	float gx1;
	float gx2;
	float gx3;
	float pre_Input;
	float pre_Output;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[Number_Of_FirstOrderFilter];
void init_FirstOrderFilters(void);
float firstOrderFilter(float input, struct firstOrderFilterData *parameters);

#endif

