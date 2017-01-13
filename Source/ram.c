
#include "include.h"

// System Status
uint8_t System_Status;

// SensorHub:

// MPU9150
MPU9150_Bias_t	MPU9150_Bias;
MPU9150_RawData_t	MPU9150_RawData;
//MPU9150_RawData.MPU_DataReady = Old_Data;
//MPU9150_RawData.Mag_DataReady = Old_Data;

MPU9150_FilteredData_t MPU9150_FilteredData; 

	
	
float selftest[6];	


// Mutex and Semaphore
xSemaphoreHandle g_pUARTSemaphore = NULL;
xSemaphoreHandle I2C3_Mutex = NULL;
xSemaphoreHandle GlobalVariable_Mutex = NULL;
xSemaphoreHandle RawDataMPU_Semaphore = NULL;

