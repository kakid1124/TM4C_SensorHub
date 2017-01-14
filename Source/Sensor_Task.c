//*************************************************//
//**** Sensor_Task.c ****//
//
// Nguyen Huynh Da Khoa - Version 1.0 - 11.01.2017 //
//
//
//*************************************************//

#include "include.h"





//*****************************************************************************
//
// Initializes the MPU9150 task.
//
//*****************************************************************************
uint32_t MPU9150TaskInit(void)
{
	
	
    // Create the MPU9150 task.
    if(xTaskCreate(MPU9150_Task, (const portCHAR *)"MPU9150", MPU9150TASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_MPU9150_TASK, NULL) != pdTRUE)
    {
        return(0);
    }

    // Success.
    return(1);
}

//*****************************************************************************
//
//	MPU9150 Task
//	
//
//*****************************************************************************
static void MPU9150_Task(void *pvParameters)
{
	static int8_t count = -1;
	MPU9150_RawData_t RawData;
	
	while(1)
	{
		if( xSemaphoreTake(RawDataMPU_Semaphore, portMAX_DELAY) == pdPASS)
		{
			count = (count + 1) % 42; // 500Hz/50 = 10Hz
			RawData.MPU_DataReady = Old_Data;
			RawData.Mag_DataReady = Old_Data;
			
			xSemaphoreTake(I2C3_Mutex, portMAX_DELAY);
			// Read Accelerometer x/y/z adc values
				MPU9150_readAccelData(RawData.accelCount);
			// Read Gyroscope x/y/z adc values
				MPU9150_readGyroData(RawData.gyroCount);
			// Read temperature adc values, Calculate Temperature in degrees Centigrade
				RawData.tempCount = MPU9150_readTempData();
				RawData.MPU_DataReady = New_Data;
				
			// Read Magnetometer Data (~10Hz)
				switch(count)
				{
					case 2:
						MPU9150_prepareMagData();
					break;
					case 20: // delay it nhat 10ms sau prepareMagData
						RawData.Mag_DataReady = MPU9150_magDataReady();
					break;
					case 25:
						if(RawData.Mag_DataReady == New_Data){
						// Read magnetometer x/y/z adc values
							MPU9150_readMagData(RawData.magCount);
						}
					break;
				}
			xSemaphoreGive(I2C3_Mutex);
			
			// Send new data to global variable
			xSemaphoreTake(GlobalVariable_Mutex, portMAX_DELAY);
				MPU9150_RawData = RawData;
			xSemaphoreGive(GlobalVariable_Mutex);
		}
	}
}


//======= Xu ly ngat MPU9150 lay du lieu ~500Hz =======//
void GPIOPortB_Handler(void){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	
	xHigherPriorityTaskWoken = pdFALSE;
	
	if(GPIO_PORTB_RIS_R & 0x04){
		GPIO_PORTB_ICR_R = 0x04;	// acknowledge flag2
		
		xSemaphoreGiveFromISR(RawDataMPU_Semaphore, &xHigherPriorityTaskWoken);
		
		//MPU9150_DataReady = true;
		//MPU9150_DataGet();				// calibrating...
	}
	
	if( xHigherPriorityTaskWoken == pdTRUE )
	{
		portSWITCH_CONTEXT();
	}
}

