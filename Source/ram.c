
#include "include.h"

// System Status
uint8_t System_Status;

// SensorHub:

// MPU9150 Variables
MPU9150_Bias_t			MPU9150_Bias;
MPU9150_FilteredData_t	MPU9150_FilteredData;



	


// Mutex and Semaphore
xSemaphoreHandle g_pUARTSemaphore = NULL;
xSemaphoreHandle I2C3_Mutex = NULL;
xSemaphoreHandle GlobalVariable_Mutex = NULL;
xSemaphoreHandle MPU9150_RawData_Semaphore = NULL;

// Queues
xQueueHandle RGBLED_Queue; // The queue that holds messages sent to the LED task.
xQueueHandle SwitchesState_Queue;

