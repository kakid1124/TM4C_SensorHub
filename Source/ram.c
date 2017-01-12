
#include "include.h"

// SensorHub:

// MPU9150
MPU9150_RawData_t	MPU9150_RawData;
  
	
float magCalibration[3], magbias[3];	// Factory mag calibration and mag bias
float gyroBias[3], accelBias[3];		// Bias corrections for gyro and accelerometer

float temperature;
	
float selftest[6];	

//

// Mutex and Semaphore
xSemaphoreHandle g_pUARTSemaphore = NULL;
xSemaphoreHandle I2C3_Semaphore = NULL;
xSemaphoreHandle RawDataMPU_Semaphore = NULL;