#include "firstOrderFilter.h"

// TAU = Filter Time Constant
// T   = Filter Sample Time
// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1/(1+A)
// GX2 = 1/(1+A)
// GX3 = (1-A)/(1+A)

#define TAU 	0.1f
#define T			0.0025f
#define A			(2*TAU/T)
#define GX1		(1/(1+A))
#define GX2		(1/(1+A))
#define GX3		((1-A)/(1+A))

// Magnetometer
#define TAU_Mag 	0.1f
#define T_Mag			0.1f
#define A_Mag			2.0f	//(2*TAU_Mag/T_Mag)
#define GX1_Mag		0.333333333f	//(1/(1+A_Mag))
#define GX2_Mag		0.333333333f	//(1/(1+A_Mag))
#define GX3_Mag		-0.333333333f	//((1-A_Mag)/(1+A_Mag))

// Accelerometer, Gyroscope
#define TAU_Accel 	0.1f
#define T_Accel			0.002f
#define A_Accel			100.0f	//(2*TAU_Mag/T_Mag)
#define GX1_Accel		0.009900990099f	//(1/(1+A_Mag))
#define GX2_Accel		0.009900990099f	//(1/(1+A_Mag))
#define GX3_Accel 	-0.9801980198f	//((1-A_Mag)/(1+A_Mag))

firstOrderFilterData_t firstOrderFilters[Number_Of_FirstOrderFilter];

void init_FirstOrderFilters(void){
	firstOrderFilters[Current_1].gx1 = GX1;
	firstOrderFilters[Current_1].gx2 = GX2;
	firstOrderFilters[Current_1].gx3 = GX3;
	firstOrderFilters[Current_1].pre_Input = 0.0f;
	firstOrderFilters[Current_1].pre_Output = 0.0f;
	
	firstOrderFilters[Current_2].gx1 = GX1;
	firstOrderFilters[Current_2].gx2 = GX2;
	firstOrderFilters[Current_2].gx3 = GX3;
	firstOrderFilters[Current_2].pre_Input = 0.0f;
	firstOrderFilters[Current_2].pre_Output = 0.0f;
	
	firstOrderFilters[Current_3].gx1 = GX1;
	firstOrderFilters[Current_3].gx2 = GX2;
	firstOrderFilters[Current_3].gx3 = GX3;
	firstOrderFilters[Current_3].pre_Input = 0.0f;
	firstOrderFilters[Current_3].pre_Output = 0.0f;
	
	firstOrderFilters[Current_4].gx1 = GX1;
	firstOrderFilters[Current_4].gx2 = GX2;
	firstOrderFilters[Current_4].gx3 = GX3;
	firstOrderFilters[Current_4].pre_Input = 0.0f;
	firstOrderFilters[Current_4].pre_Output = 0.0f;
	
	firstOrderFilters[Magneto_X].gx1 = GX1_Mag;
	firstOrderFilters[Magneto_X].gx2 = GX2_Mag;
	firstOrderFilters[Magneto_X].gx3 = GX3_Mag;
	firstOrderFilters[Magneto_X].pre_Input = 0.0f;
	firstOrderFilters[Magneto_X].pre_Output = 0.0f;
	
	firstOrderFilters[Magneto_Y].gx1 = GX1_Mag;
	firstOrderFilters[Magneto_Y].gx2 = GX2_Mag;
	firstOrderFilters[Magneto_Y].gx3 = GX3_Mag;
	firstOrderFilters[Magneto_Y].pre_Input = 0.0f;
	firstOrderFilters[Magneto_Y].pre_Output = 0.0f;
	
	firstOrderFilters[Magneto_Z].gx1 = GX1_Mag;
	firstOrderFilters[Magneto_Z].gx2 = GX2_Mag;
	firstOrderFilters[Magneto_Z].gx3 = GX3_Mag;
	firstOrderFilters[Magneto_Z].pre_Input = 0.0f;
	firstOrderFilters[Magneto_Z].pre_Output = 0.0f;
	
	firstOrderFilters[SRF05_Sensor].gx1 = 1.0f/11.0f;
	firstOrderFilters[SRF05_Sensor].gx2 = 1.0f/11.0f;
	firstOrderFilters[SRF05_Sensor].gx3 = -9.0f/11.0f;
	firstOrderFilters[SRF05_Sensor].pre_Input = 0.023f;
	firstOrderFilters[SRF05_Sensor].pre_Output = 0.023f;

	firstOrderFilters[Accel_X].gx1 = GX1_Accel;
	firstOrderFilters[Accel_X].gx2 = GX2_Accel;
	firstOrderFilters[Accel_X].gx3 = GX3_Accel;
	firstOrderFilters[Accel_X].pre_Input = 0.0f;
	firstOrderFilters[Accel_X].pre_Output = 0.0f;
	
	firstOrderFilters[Accel_Y].gx1 = GX1_Accel;
	firstOrderFilters[Accel_Y].gx2 = GX2_Accel;
	firstOrderFilters[Accel_Y].gx3 = GX3_Accel;
	firstOrderFilters[Accel_Y].pre_Input = 0.0f;
	firstOrderFilters[Accel_Y].pre_Output = 0.0f;
	
	firstOrderFilters[Accel_Z].gx1 = GX1_Accel;
	firstOrderFilters[Accel_Z].gx2 = GX2_Accel;
	firstOrderFilters[Accel_Z].gx3 = GX3_Accel;
	firstOrderFilters[Accel_Z].pre_Input = 16500.0f;
	firstOrderFilters[Accel_Z].pre_Output = 16500.0f;
	
	firstOrderFilters[Gyro_X].gx1 = GX1_Accel;
	firstOrderFilters[Gyro_X].gx2 = GX2_Accel;
	firstOrderFilters[Gyro_X].gx3 = GX3_Accel;
	firstOrderFilters[Gyro_X].pre_Input = 0.0f;
	firstOrderFilters[Gyro_X].pre_Output = 0.0f;
	
	firstOrderFilters[Gyro_Y].gx1 = GX1_Accel;
	firstOrderFilters[Gyro_Y].gx2 = GX2_Accel;
	firstOrderFilters[Gyro_Y].gx3 = GX3_Accel;
	firstOrderFilters[Gyro_Y].pre_Input = 0.0f;
	firstOrderFilters[Gyro_Y].pre_Output = 0.0f;
	
	firstOrderFilters[Gyro_Z].gx1 = GX1_Accel;
	firstOrderFilters[Gyro_Z].gx2 = GX2_Accel;
	firstOrderFilters[Gyro_Z].gx3 = GX3_Accel;
	firstOrderFilters[Gyro_Z].pre_Input = 0.0f;
	firstOrderFilters[Gyro_Z].pre_Output = 0.0f;
	
	firstOrderFilters[AccelZ_HP].gx1 = 2.0f/3.0f;
	firstOrderFilters[AccelZ_HP].gx2 = -2.0f/3.0f;
	firstOrderFilters[AccelZ_HP].gx3 = -1.0f/3.0f;
	firstOrderFilters[AccelZ_HP].pre_Input = 0.0f;
	firstOrderFilters[AccelZ_HP].pre_Output = 0.0f;
}

float firstOrderFilter(float input, struct firstOrderFilterData *parameters){
	float output;
	
	output = parameters->gx1 * input +
					 parameters->gx2 * parameters->pre_Input -
					 parameters->gx3 * parameters->pre_Output;
	
	parameters->pre_Input = input;
	parameters->pre_Output = output;
	
	return output;
}
