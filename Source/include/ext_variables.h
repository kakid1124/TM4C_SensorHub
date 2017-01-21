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

					
					
// Stack size in words
#define MAINTASKSTACKSIZE       128
#define MPU9150TASKSTACKSIZE		128
#define LEDTASKSTACKSIZE        128
#define SWITCHTASKSTACKSIZE     128


#define LED_ITEM_SIZE           sizeof(LED_Data_t)
#define LED_QUEUE_SIZE          3


// The priorities of the various tasks.
#define PRIORITY_MAIN_TASK			1
#define PRIORITY_MPU9150_TASK		1
#define PRIORITY_SWITCH_TASK    2
#define PRIORITY_LED_TASK       1

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


// LED Toggle
typedef struct {
	uint8_t LED_Code;	// color
	uint8_t duty;		// high percent (0~100%)
	float freq;			// Frequency (Hz)
} LED_Data_t;



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

// Queues
extern xQueueHandle RGBLED_Queue; // The queue that holds messages sent to the LED task.
extern xQueueHandle SwitchesState_Queue;

#endif
