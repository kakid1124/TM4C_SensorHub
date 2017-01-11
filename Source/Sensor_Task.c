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
	
}

//*****************************************************************************
//
//	MPU9150 Task
//	
//
//*****************************************************************************
static void MPU9150_Task(void *pvParameters)
{
	
}


//======= Xu ly ngat MPU9150 lay du lieu ~500Hz =======//
void GPIOPortB_Handler(void){
	if(GPIO_PORTB_RIS_R & 0x04){
		GPIO_PORTB_ICR_R = 0x04;	// acknowledge flag2
		MPU9150_DataReady = true;
		//MPU9150_DataGet();				// calibrating...
	}
}

void MPU9150_DataGet(void){
		magcount = (magcount + 1)%42; // 500Hz/50 = 10Hz
		
		// Read Accelerometer x/y/z adc values
		MPU9150.readAccelData(MPU9150.accelCount);  
		Accel_Data[0] = firstOrderFilter((float)MPU9150.accelCount[0], &firstOrderFilters[Accel_X]);
		Accel_Data[1] = firstOrderFilter((float)MPU9150.accelCount[1], &firstOrderFilters[Accel_Y]);
		Accel_Data[2] = firstOrderFilter((float)MPU9150.accelCount[2], &firstOrderFilters[Accel_Z]);
		
		// Read Gyroscope x/y/z adc values
    MPU9150.readGyroData(MPU9150.gyroCount);
		Gyro_Data[0] = firstOrderFilter((float)MPU9150.gyroCount[0], &firstOrderFilters[Gyro_X]);
		Gyro_Data[1] = firstOrderFilter((float)MPU9150.gyroCount[1], &firstOrderFilters[Gyro_Y]);
		Gyro_Data[2] = firstOrderFilter((float)MPU9150.gyroCount[2], &firstOrderFilters[Gyro_Z]);
		
		// Read temperature adc values, Calculate Temperature in degrees Centigrade
		MPU9150.tempCount = MPU9150.readTempData();
		MPU9150.temperature = ((float) MPU9150.tempCount) / 340.0f + 36.53f;
		
		// Read Magnetometer Data (~10Hz)
		if(magcount == 2){
			MPU9150.prepareMagData();
		}
		else if(magcount == 20){	// delay it nhat 10ms sau prepareMagData
			MPU9150.newMagData = MPU9150.magDataReady();
		}
		else if(magcount == 25){
			if(MPU9150.newMagData == true){
				// Read magnetometer x/y/z adc values
				MPU9150.readMagData(MPU9150.magCount);
			}
		}
		else if(magcount == 30){
			if(MPU9150.newMagData == true){  // doi truc: X'=Y; Y'=X; Z'=-Z
				MagData_Data[0] = firstOrderFilter((float)MPU9150.magCount[1] * MPU9150.magCalibration[1], &firstOrderFilters[Magneto_X]);
				MagData_Data[1] = firstOrderFilter((float)MPU9150.magCount[0] * MPU9150.magCalibration[0], &firstOrderFilters[Magneto_Y]);
				MagData_Data[2] = firstOrderFilter((float)MPU9150.magCount[2] *(-1.0f)*MPU9150.magCalibration[2], &firstOrderFilters[Magneto_Z]);
				
				MPU9150.newMagData = false;
			}
		}
}
