#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

//*****************************************************************************
//
// Prototypes for the Sensor task.
//
//*****************************************************************************
extern uint32_t MPU9150TaskInit(void);
extern void MPU9150_Task(void *pvParameters);
	
#endif // __SENSOR_TASK_H__
