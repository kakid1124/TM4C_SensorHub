//*************************************************//
//**** Tiva C - FreeRTOS Project ****//
//
// Nguyen Huynh Da Khoa - Version 1.0 - 10.01.2017 //
//
// System clock: 8 ~ 40MHz
//
//*************************************************//

#include "include.h"

//**** Function Prototype ****//
#ifdef __cplusplus
extern "C" {
#endif

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void System_Init(void);
	
#ifdef __cplusplus
}
#endif





//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//**************************************//
//
//************ Main Program ************//	
//
//**************************************//

int main()
{
	System_Init();
	
	UART_OutString("\n\nWelcome to the FreeRTOS Demo...\n");
	
    // Create mutex and semaphore
    g_pUARTSemaphore = xSemaphoreCreateMutex();
	I2C3_Mutex = xSemaphoreCreateMutex();
	GlobalVariable_Mutex = xSemaphoreCreateMutex();
	
	vSemaphoreCreateBinary(RawDataMPU_Semaphore);

	
    //
    // Create the LED task.
    //
    if(LEDTaskInit() != 0)
    {

        while(1)
        {
        }
    }

    //
    // Create the switch task.
    //
    if(SwitchTaskInit() != 0)
    {

        while(1)
        {
        }
    }

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
		
		while(1)
		{
		}
}


//*************************************************//
//**** Systems_Init ****//
//
//*************************************************//
void System_Init(void)
{
#ifdef System_Clock_40MHz
	PLL_Init();			// System Clock: 40MHz
	UART_Init();		// UART: 115200 bps
	Timer0A_Delay_Init();
#elif System_Clock_8MHz
	PLL_Init_8MHz(); 	// System Clock: 8MHz
	UART_Init_9600bps();// UART: 9600 bps
	Timer0A_Delay_Init_8MHz();
#endif	

//	PortF_Init();	
//	EnableInterrupts();
	
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();
	
	
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
}

//*****************************************************************************
//
//	MAIN Task
//	
//
//*****************************************************************************
static void Main_Task(void *pvParameters)
{
	portTickType ui16LastTime;
	static uint8_t count;
	
	while(1)
	{
		
		switch(System_Status)
		{
			case System_Ready:	
				// Get the current tick count.
				count = 0;
				ui16LastTime = xTaskGetTickCount();				
				while(1)
				{
					count = (count+1)%40;
					
					// First Order Filter
					xSemaphoreTake(GlobalVariable_Mutex, portMAX_DELAY);
					if(MPU9150_RawData.MPU_DataReady)
					{
						MPU9150_RawData.MPU_DataReady = Old_Data;
						
						// Accelerometer x/y/z adc values
						MPU9150_FilteredData.Accel[0] = firstOrderFilter((float)MPU9150_RawData.accelCount[0], &firstOrderFilters[Accel_X]);
						MPU9150_FilteredData.Accel[1] = firstOrderFilter((float)MPU9150_RawData.accelCount[1], &firstOrderFilters[Accel_Y]);
						MPU9150_FilteredData.Accel[2] = firstOrderFilter((float)MPU9150_RawData.accelCount[2], &firstOrderFilters[Accel_Z]);
					
						// Gyroscope x/y/z adc values
						MPU9150_FilteredData.Gyro[0] = firstOrderFilter((float)MPU9150_RawData.gyroCount[0], &firstOrderFilters[Gyro_X]);
						MPU9150_FilteredData.Gyro[1] = firstOrderFilter((float)MPU9150_RawData.gyroCount[1], &firstOrderFilters[Gyro_Y]);
						MPU9150_FilteredData.Gyro[2] = firstOrderFilter((float)MPU9150_RawData.gyroCount[2], &firstOrderFilters[Gyro_Z]);
						
						// Temperature adc values, Calculate Temperature in degrees Centigrade
						MPU9150_FilteredData.temperature = ((float) MPU9150_RawData.tempCount) / 340.0f + 36.53f;
						
						if(MPU9150_RawData.Mag_DataReady)
						{
							MPU9150_RawData.Mag_DataReady = Old_Data;
							
							// Magnetometer x/y/z adc values
							// doi truc: X'=Y; Y'=X; Z'=-Z
							MPU9150_FilteredData.Magneto[0] = firstOrderFilter((float)MPU9150_RawData.magCount[1] * MPU9150_Bias.magCalib[1], &firstOrderFilters[Magneto_X]);
							MPU9150_FilteredData.Magneto[1] = firstOrderFilter((float)MPU9150_RawData.magCount[0] * MPU9150_Bias.magCalib[0], &firstOrderFilters[Magneto_Y]);
							MPU9150_FilteredData.Magneto[2] = firstOrderFilter((float)MPU9150_RawData.magCount[2] *(-1.0f)*MPU9150_Bias.magCalib[2], &firstOrderFilters[Magneto_Z]);
						}
					}
					xSemaphoreGive(GlobalVariable_Mutex);
					
					// Uoc Luong Goc Quay
					
					/*
					current_Time = WTIMER0_TAR_R;
					delta_Time = (previous_Time - current_Time)&0xFFFFFFFF;
					delta_T = delta_Time * 0.000001f;										// [micro second]*10^(-6) --> [second]
					previous_Time = current_Time;	
			
					MPU9150_Compute_TCBias(&MPU9150.temperature);
					pAccel[0] = (Accel_Data[0] - accel_BiasMPU[0] - accel_TCBias[0]) * accel_ScaleFactorMPU[0];  // [m/s^2]
					pAccel[1] = (Accel_Data[1] - accel_BiasMPU[1] - accel_TCBias[1]) * accel_ScaleFactorMPU[1];
					pAccel[2] = (Accel_Data[2] - accel_BiasMPU[2] - accel_TCBias[2]) * accel_ScaleFactorMPU[2];
			
					pGyro[0] = (Gyro_Data[0] - gyro_RTBias[0] - gyro_TCBias[0]) * MPU9150.gRes * D2R; // [rad/s]
					pGyro[1] = (Gyro_Data[1] - gyro_RTBias[1] - gyro_TCBias[1]) * MPU9150.gRes * D2R; // [rad/s]
					pGyro[2] = (Gyro_Data[2] - gyro_RTBias[2] - gyro_TCBias[2]) * MPU9150.gRes * D2R; // [rad/s]
			
					QUEST((float*)pAccel, (float*)pGyro, (float*)pMag, (float*)pEulers, (float*)pQuaternion, &delta_T);
					*/
					
					// Magnetometer
					if(count == 39)
					{
						/* Mag_Transformation((float*)MagData_Data);
						Mag_vector_length_stabilasation();
						pMag[0] = Mag_calibrated[0]*MPU9150.mRes;	// [milliGauss]
						pMag[1] = Mag_calibrated[1]*MPU9150.mRes;
						pMag[2] = Mag_calibrated[2]*MPU9150.mRes;
					*/
					}
					
					if(System_Status != System_Ready) break;
						
					// Frequency: 400Hz --> T=2.5ms
					vTaskDelayUntil(&ui16LastTime, (2.5f) / portTICK_RATE_MS);
				}
			break;
			
			case Calib_Mode:
			
			break;
			
			case Initialization:
			
			break;
			
			case Hardware_Error:
			
			break;
		}
		
	}
}



