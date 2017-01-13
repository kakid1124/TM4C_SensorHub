#ifndef EXT_VARIABLES_H_
#define EXT_VARIABLES_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// System:
enum System_Status {Initialization,
					Calib_Mode,
					System_Ready,
					ManualFlight_Mode,
					AutoFlight_Mode,
					Hardware_Error
					};
					

// SensorHub:
#define New_Data 1
#define Old_Data 0
typedef struct {
	float Accel[3];	// Bias corrections for accelerometer
	float Gyro[3];	// Bias corrections for gyro
	float Mag[3];	// Factory mag calibration and mag bias
	float magCalib[3];
	float mRes;
} MPU9150_Bias_t;

typedef struct {
	int16_t accelCount[3];	// Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
	uint8_t MPU_DataReady;
	uint8_t Mag_DataReady;
} MPU9150_RawData_t;

typedef struct {
	float Accel[3];
	float Gyro[3];
	float Magneto[3];
	float temperature;
} MPU9150_FilteredData_t;


// Extern Variables:
extern uint8_t System_Status;

extern MPU9150_Bias_t MPU9150_Bias;
extern MPU9150_RawData_t MPU9150_RawData;
extern MPU9150_FilteredData_t MPU9150_FilteredData;

// Mutex and Semaphore
extern xSemaphoreHandle g_pUARTSemaphore;
extern xSemaphoreHandle I2C3_Mutex;
extern xSemaphoreHandle GlobalVariable_Mutex;
extern xSemaphoreHandle RawDataMPU_Semaphore;

#endif
