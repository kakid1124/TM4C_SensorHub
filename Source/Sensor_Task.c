//*************************************************//
//**** Sensor_Task.c ****//
//
// Nguyen Huynh Da Khoa - Version 1.0 - 11.01.2017 //
//
//
//*************************************************//

#include "include.h"


//======= Xu ly ngat MPU9150 lay du lieu ~500Hz =======//
void GPIOPortB_Handler(void){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	
	xHigherPriorityTaskWoken = pdFALSE;
	
	if(GPIO_PORTB_RIS_R & 0x04){
		GPIO_PORTB_ICR_R = 0x04;	// acknowledge flag2
		
		xSemaphoreGiveFromISR(MPU9150_RawData_Semaphore, &xHigherPriorityTaskWoken);
		
		// Old style:
		//MPU9150_DataReady = true;
		//MPU9150_DataGet();				// for calibrating...
	}
	
	if( xHigherPriorityTaskWoken == pdTRUE )
	{
		vTaskSwitchContext();
	}
}


//*****************************************************************************
//
//		MPU9150 QUEST Filter Task
//	1. Read raw datas from MPU: ~500Hz
//	2. Apply QUEST filter to create Euler Angles
//
//*****************************************************************************
static void MPU9150_QUESTFilter_Task(void *pvParameters)
{
	static int8_t count = -1, quest_freqdev = -1;
	MPU9150_RawData_t			RawData;
	MPU9150_preFilteredData_t	FirstOrderFilteredData;
	MPU9150_FilteredData_t		QUEST_FilteredData;
	
	static uint32_t current_Time, previous_Time, delta_T;
	float delta_Ts;

	while(1)
	{
		if( xSemaphoreTake(MPU9150_RawData_Semaphore, portMAX_DELAY) == pdPASS)
		{		
		// Get new raw data from MPU:
		
			xSemaphoreTake(I2C3_Mutex, portMAX_DELAY);
			// Read Accelerometer x/y/z adc values
				MPU9150_readAccelData(RawData.accelCount);
			// Read Gyroscope x/y/z adc values
				MPU9150_readGyroData(RawData.gyroCount);
			// Read temperature adc values, Calculate Temperature in degrees Centigrade
				RawData.tempCount = MPU9150_readTempData();
				
			// Read Magnetometer Data (~10Hz)
				count = (count + 1) % 42; // 500Hz/50 = 10Hz
				RawData.Mag_DataReady = Old_Data;
				
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
		
		// First Order Filter 
			// Accelerometer x/y/z adc values
			FirstOrderFilteredData.Accel[0] = firstOrderFilter((float)RawData.accelCount[0], &firstOrderFilters[Accel_X]);
			FirstOrderFilteredData.Accel[1] = firstOrderFilter((float)RawData.accelCount[1], &firstOrderFilters[Accel_Y]);
			FirstOrderFilteredData.Accel[2] = firstOrderFilter((float)RawData.accelCount[2], &firstOrderFilters[Accel_Z]);
					
			// Gyroscope x/y/z adc values
			FirstOrderFilteredData.Gyro[0] = firstOrderFilter((float)RawData.gyroCount[0], &firstOrderFilters[Gyro_X]);
			FirstOrderFilteredData.Gyro[1] = firstOrderFilter((float)RawData.gyroCount[1], &firstOrderFilters[Gyro_Y]);
			FirstOrderFilteredData.Gyro[2] = firstOrderFilter((float)RawData.gyroCount[2], &firstOrderFilters[Gyro_Z]);
						
			// Temperature adc values, Calculate Temperature in degrees Centigrade
			FirstOrderFilteredData.temperature = ((float) RawData.tempCount) / 340.0f + 36.53f;
			
			if(RawData.Mag_DataReady)
			{						
				// Magnetometer x/y/z adc values
				// doi truc: X'=Y; Y'=X; Z'=-Z
				FirstOrderFilteredData.Magneto[0] = firstOrderFilter((float)RawData.magCount[1] * MPU9150_Bias.magCalib[1], &firstOrderFilters[Magneto_X]);
				FirstOrderFilteredData.Magneto[1] = firstOrderFilter((float)RawData.magCount[0] * MPU9150_Bias.magCalib[0], &firstOrderFilters[Magneto_Y]);
				FirstOrderFilteredData.Magneto[2] = firstOrderFilter((float)RawData.magCount[2] *(-1.0f)*MPU9150_Bias.magCalib[2], &firstOrderFilters[Magneto_Z]);
			}
		
		// QUEST Filter: apply frequency devider
			quest_freqdev = (quest_freqdev + 1) % 4; // ~100Hz
			if(quest_freqdev == 1)
			{			
				// Magnetometer stabilasation
				if(RawData.Mag_DataReady)
				{
					Mag_Transformation((float*)FirstOrderFilteredData.Magneto);
					Mag_vector_length_stabilasation();
					QUEST_FilteredData.Magneto[0] = Mag_calibrated[0]*MPU9150_Bias.mRes;	// [milliGauss]
					QUEST_FilteredData.Magneto[1] = Mag_calibrated[1]*MPU9150_Bias.mRes;
					QUEST_FilteredData.Magneto[2] = Mag_calibrated[2]*MPU9150_Bias.mRes;
				}
				
				// Accel and Gyro compute bias
				MPU9150_Compute_TCBias(&FirstOrderFilteredData.temperature);
				QUEST_FilteredData.Accel[0] = (FirstOrderFilteredData.Accel[0] - accel_BiasMPU[0] - accel_TCBias[0]) * accel_ScaleFactorMPU[0];  // [m/s^2]
				QUEST_FilteredData.Accel[1] = (FirstOrderFilteredData.Accel[1] - accel_BiasMPU[1] - accel_TCBias[1]) * accel_ScaleFactorMPU[1];
				QUEST_FilteredData.Accel[2] = (FirstOrderFilteredData.Accel[2] - accel_BiasMPU[2] - accel_TCBias[2]) * accel_ScaleFactorMPU[2];
			
				QUEST_FilteredData.Gyro[0] = (FirstOrderFilteredData.Gyro[0] - gyro_RTBias[0] - gyro_TCBias[0]) * MPU9150_Bias.gRes * D2R; // [rad/s]
				QUEST_FilteredData.Gyro[1] = (FirstOrderFilteredData.Gyro[1] - gyro_RTBias[1] - gyro_TCBias[1]) * MPU9150_Bias.gRes * D2R; // [rad/s]
				QUEST_FilteredData.Gyro[2] = (FirstOrderFilteredData.Gyro[2] - gyro_RTBias[2] - gyro_TCBias[2]) * MPU9150_Bias.gRes * D2R; // [rad/s]
				
				// Apply QUEST Filter
				current_Time = WTIMER0_TAR_R;
				delta_T = (previous_Time - current_Time)&0xFFFFFFFF;
				delta_Ts = delta_T * 0.000001f;		// [micro second]*10^(-6) --> [second]
				previous_Time = current_Time;
				
				QUEST((float*)QUEST_FilteredData.Accel, (float*)QUEST_FilteredData.Gyro, (float*)QUEST_FilteredData.Magneto, (float*)QUEST_FilteredData.Euler, (float*)QUEST_FilteredData.Quaternion, &delta_Ts);
				
				// Send new data to global variable
				xSemaphoreTake(GlobalVariable_Mutex, portMAX_DELAY);
				MPU9150_FilteredData = QUEST_FilteredData;
				xSemaphoreGive(GlobalVariable_Mutex);
			}
		}
	}
}




//*****************************************************************************
//
//	Initializes the MPU9150 task.
//	
//*****************************************************************************
uint32_t MPU9150TaskInit(void)
{
	//**** MPU9150 (I2C3): PD0 --> SCL, PD1 --> SDA, PB2 --> INT ****//					
	I2C3_Init();
	MotionInt_Init();
	
	if(MPU9150_setup() == pdTRUE)
	{
		// MPU9150 is ready
		UART_OutString((char *)"\n\rMPU9150 is ready for access...\n\r");
		init_FirstOrderFilters();
		
		// Create the MPU9150 task.
		I2C3_Mutex = xSemaphoreCreateMutex();
		vSemaphoreCreateBinary(MPU9150_RawData_Semaphore);
		
		if(xTaskCreate(MPU9150_QUESTFilter_Task, (const portCHAR *)"MPU9150", MPU9150_QUESTFilter_STACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_MPU9150_QUESTFilter_TASK, NULL) != pdTRUE)
		{
			return(0);
		}

		// Success.
		return(1);
	}
	else
	{
		// MPU9150 is bad
		UART_OutString((char *)"\n\rMPU9150's connection is bad!\n\r");
		return(0);
	}
}

