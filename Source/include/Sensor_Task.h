#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

#include <stdint.h>

//*****************************************************************************
//
// Prototypes for the Sensor task.
//
//*****************************************************************************
void GPIOPortB_Handler(void);
extern uint32_t MPU9150TaskInit(void);

	
#endif // __SENSOR_TASK_H__
