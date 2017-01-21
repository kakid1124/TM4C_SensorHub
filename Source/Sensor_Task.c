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
		vTaskSwitchContext();
	}
}

//*****************************************************************************
//
// Initializes the MPU9150 task.
//
//*****************************************************************************
uint32_t MPU9150TaskInit(void)
{
	//**** MPU9150 (I2C3): PD0 --> SCL, PD1 --> SDA, PB2 --> INT ****//					
	I2C3_Init();
	MotionInt_Init();
	MPU9150_setup();

/*	
	// Read the WHO_AM_I register, this is a good test of communication
	uint8_t whoami = MPU9150.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);
	if((Battery_Ready)&&(whoami == 0x68)){
		MPU9150.selftest(MPU9150.SelfTest);
		
		UART_OutString((char *)"accel x-axis self test: ");
		UART_OutFloat(MPU9150.SelfTest[0]);
		UART_OutString((char *)"% of factory value\n");
		UART_OutString((char *)"accel y-axis self test: ");
		UART_OutFloat(MPU9150.SelfTest[1]);
		UART_OutString((char *)"% of factory value\n");
		UART_OutString((char *)"accel z-axis self test: ");
		UART_OutFloat(MPU9150.SelfTest[2]);
		UART_OutString((char *)"% of factory value\n");
		
		UART_OutString((char *)"gyro x-axis self test: ");
		UART_OutFloat(MPU9150.SelfTest[3]);
		UART_OutString((char *)"% of factory value\n");
		UART_OutString((char *)"gyro y-axis self test: ");
		UART_OutFloat(MPU9150.SelfTest[4]);
		UART_OutString((char *)"% of factory value\n");
		UART_OutString((char *)"gyro z-axis self test: ");
		UART_OutFloat(MPU9150.SelfTest[5]);
		UART_OutString((char *)"% of factory value\n");
		
		Delay_ms(100);
		
		// Reset registers to default in preparation for device calibration
		// Calibrate gyro and accelerometers, load biases in bias registers 
		MPU9150.resetMPU9150();
    MPU9150.calibrate(MPU9150.gyroBias, MPU9150.accelBias); 
		
		UART_OutString((char *)"x gyro bias = ");
		UART_OutFloat(MPU9150.gyroBias[0]);
		UART_OutString((char *)"\n\r");
		UART_OutString((char *)"y gyro bias = ");
		UART_OutFloat(MPU9150.gyroBias[1]);
		UART_OutString((char *)"\n\r");
		UART_OutString((char *)"z gyro bias = ");
		UART_OutFloat(MPU9150.gyroBias[2]);
		UART_OutString((char *)"\n\r");
		
		UART_OutString((char *)"x accel bias = ");
		UART_OutFloat(MPU9150.accelBias[0]);
		UART_OutString((char *)"\n\r");
		UART_OutString((char *)"y accel bias = ");
		UART_OutFloat(MPU9150.accelBias[1]);
		UART_OutString((char *)"\n\r");
		UART_OutString((char *)"z accel bias = ");
		UART_OutFloat(MPU9150.accelBias[2]);
		UART_OutString((char *)"\n\r");
		
		Delay_ms(100);
		
		// MPU9150 initialize
		// Calibrate Magnetometer
		MPU9150.init(); 
		MPU9150.initAK8975A(MPU9150.magCalibration);
		
		UART_OutString((char *)"x magneto calibration = ");
		UART_OutFloat(MPU9150.magCalibration[0]);
		UART_OutString((char *)"\n\r");
		UART_OutString((char *)"y magneto calibration = ");
		UART_OutFloat(MPU9150.magCalibration[1]);
		UART_OutString((char *)"\n\r");
		UART_OutString((char *)"z magneto calibration = ");
		UART_OutFloat(MPU9150.magCalibration[2]);
		UART_OutString((char *)"\n\r");
		
		// MPU9150 is ready
		UART_OutString((char *)"MPU9150 is ready for active data mode...\n\r");
		MPU9150_Ready = true;	
	*/
	
    // Create the MPU9150 task.
    if(xTaskCreate(MPU9150_Task, (const portCHAR *)"MPU9150", MPU9150TASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_MPU9150_TASK, NULL) != pdTRUE)
    {
        return(0);
    }

    // Success.
    return(1);
}

