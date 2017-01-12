#ifndef EXT_VARIABLES_H_
#define EXT_VARIABLES_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// SensorHub:
typedef struct {
	int16_t accelCount[3];	// Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
} MPU9150_RawData_t;




// Extern Variables:
extern MPU9150_RawData_t MPU9150_RawData;

// Mutex and Semaphore
extern xSemaphoreHandle g_pUARTSemaphore;
extern xSemaphoreHandle I2C3_Semaphore;
extern xSemaphoreHandle RawDataMPU_Semaphore;

#endif
