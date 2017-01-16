#include "calibration.h"
#include "MPU9150.h"
#include "Timer0A.h"
#include "UART.h"

//calibration_matrix[3][3] is the transformation matrix
	const float calibration_matrix[3][3] = 
  {
    {16.704f, 0.691f, 0.105f},
    {-0.587f, 16.206f, 0.315f},
    {-0.202f, -0.846f, 8.714f}  
  };
//bias[3] is the bias
  const float bias[3] = 
  {
    47.637f,
    -42.322f,
    -58.512f
  }; 
	
	
bool MPU9150_calibrating;
float accelSum[3] = {0.0,0.0,0.0};
float gyroSum[3] = {0.0,0.0,0.0};
float accel_OneG;

float Mag_calibrated[3];
bool Mag_scaler_flag = false;
float Mag_scaler;
float Mag_normal_vector_length;

float accel_TCBias[3], gyro_TCBias[3];

//float accel_BiasMPU[3] = {-311.226f, -179.510f, -16240.245f};
//float accel_ScaleFactorMPU[3] = {0.000600891f, 0.000598212f, 0.000603036f};
float accel_BiasMPU[3] = {124.864f, -84.713f, -16297.550f};
float accel_ScaleFactorMPU[3] = {0.000600849f, 0.000598168f, 0.000603052f};

float gyro_RTBias[3];
//float accel_TCBias_Slope[3] 		= {-14.588f, 2.803f, -6.947f};			//{-13.715f, 2.194f, -10.237f};
//float	accel_TCBias_Intercept[3] = {519.557f, -91.795f, 16635.744f};	//{533.694f, -129.787f, 8587.426f};
//float gyro_TCBias_Slope[3]			= {-0.231f, -3.695f, 0.189f};				//{0.084f, -3.115f, 0.291f};
//float gyro_TCBias_Intercept[3]  = {-1.127f, 118.84f, -3.978f};			//{-6.353f, 108.499f, -7.841f};
float accel_TCBias_Slope[3] 		= {-10.724f, -4.240f, -14.071f};
float	accel_TCBias_Intercept[3] = {412.586f, 139.991f, 16966.976f};
float gyro_TCBias_Slope[3]			= {-1.540f, -0.095f, -2.507f};
float gyro_TCBias_Intercept[3]  = {49.749f, 0.390f, 84.081f};	

//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
void Mag_Transformation(float * uncalibrated_values){
	int8_t i,j;
	float result[3] = {0, 0, 0};
//calculation
  for (i=0; i<3; ++i)
		uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  
  for (i=0; i<3; ++i)
    for (j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
	
  for (i=0; i<3; ++i) 
		Mag_calibrated[i] = result[i];	
}

//vector_length_stabilasation() - is the function of the magnetometer vector length stabilasation (stabilisation of the sphere radius)
void Mag_vector_length_stabilasation(void){
  //calculate the normal vector length
  if (Mag_scaler_flag == false)
  {
    //getHeading();
    Mag_normal_vector_length = sqrtf(Mag_calibrated[0]*Mag_calibrated[0] + Mag_calibrated[1]*Mag_calibrated[1] + Mag_calibrated[2]*Mag_calibrated[2]);
    Mag_scaler_flag = true;
  } 
  //calculate the current scaler
  Mag_scaler = Mag_normal_vector_length/sqrtf(Mag_calibrated[0]*Mag_calibrated[0] + Mag_calibrated[1]*Mag_calibrated[1] + Mag_calibrated[2]*Mag_calibrated[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  Mag_calibrated[0] = Mag_calibrated[0]*Mag_scaler;
  Mag_calibrated[1] = Mag_calibrated[1]*Mag_scaler;
  Mag_calibrated[2] = Mag_calibrated[2]*Mag_scaler;
}

void MPU9150_TCBias_Measurement(float * accel_raw, float * gyro_raw, int16_t * temp_raw){
	uint16_t index;
	uint16_t numberOfSamples = 5000;

	float accel_Bias1[3] = {0,0,0}, gyro_Bias1[3]={0,0,0}, Temperature1 = 0.0f;
	float accel_Bias2[3] = {0,0,0}, gyro_Bias2[3]={0,0,0}, Temperature2 = 0.0f;

	UART_OutString((char *)"\nMPU910 Calibration:\n");
	UART_OutString((char *)"Begin 1st MPU9150 Measurements...\n");
	for(index = 0; index < numberOfSamples; index++){

		accel_Bias1[0] += (float)accel_raw[0];
		accel_Bias1[1] += (float)accel_raw[1];
		accel_Bias1[2] += (float)(accel_raw[2] - 8192.0f);
		
		gyro_Bias1[0] += (float)gyro_raw[0];
		gyro_Bias1[1] += (float)gyro_raw[1];	
		gyro_Bias1[2] += (float)gyro_raw[2];
			
		Temperature1 += (float)(*temp_raw);

		Delay_ms(4);
	}
	
	accel_Bias1[0] /= (float)numberOfSamples;
	accel_Bias1[1] /= (float)numberOfSamples;
	accel_Bias1[2] /= (float)numberOfSamples;
	gyro_Bias1[0] /= (float)numberOfSamples;
	gyro_Bias1[1] /= (float)numberOfSamples;
	gyro_Bias1[2] /= (float)numberOfSamples;
	Temperature1 /= (float)numberOfSamples;
	
	Temperature1 = Temperature1 / 340.0f + 36.53f;
	
	UART_OutString((char *)"\nTemperature Reading: ");
	UART_OutFloat(Temperature1);
	UART_OutString((char *)"\nAccel Bias: ");
	UART_OutFloat(accel_Bias1[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_Bias1[1]);	
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_Bias1[2]);
	UART_OutString((char *)"\nGyro Bias: ");
	UART_OutFloat(gyro_Bias1[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(gyro_Bias1[1]);	
	UART_OutString((char *)" : ");
	UART_OutFloat(gyro_Bias1[2]);
	UART_OutString((char *)"\nEnd 1st MPU9150 Measurements\n");
	
	UART_OutString((char *)"Waiting for 15 minutes for MPU9150 temp to rise...\n");
	Delay_ms(900000);
	
	UART_OutString((char *)"Begin 2nd MPU9150 Measurements...\n");
	for(index = 0; index < numberOfSamples; index++){

		accel_Bias2[0] += (float)accel_raw[0];
		accel_Bias2[1] += (float)accel_raw[1];
		accel_Bias2[2] += (float)(accel_raw[2] - 8192.0f);
		
		gyro_Bias2[0] += (float)gyro_raw[0];
		gyro_Bias2[1] += (float)gyro_raw[1];	
		gyro_Bias2[2] += (float)gyro_raw[2];
			
		Temperature2 += (float)(*temp_raw);

		Delay_ms(4);
	}
	
	accel_Bias2[0] /= (float)numberOfSamples;
	accel_Bias2[1] /= (float)numberOfSamples;
	accel_Bias2[2] /= (float)numberOfSamples;
	gyro_Bias2[0] /= (float)numberOfSamples;
	gyro_Bias2[1] /= (float)numberOfSamples;
	gyro_Bias2[2] /= (float)numberOfSamples;
	Temperature2 /= (float)numberOfSamples;

	Temperature2 = Temperature2 / 340.0f + 36.53f;
	
	UART_OutString((char *)"\nTemperature Reading: ");
	UART_OutFloat(Temperature2);
	UART_OutString((char *)"\nAccel Bias: ");
	UART_OutFloat(accel_Bias2[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_Bias2[1]);	
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_Bias2[2]);
	UART_OutString((char *)"\nGyro Bias: ");
	UART_OutFloat(gyro_Bias2[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(gyro_Bias2[1]);	
	UART_OutString((char *)" : ");
	UART_OutFloat(gyro_Bias2[2]);
	UART_OutString((char *)"\nEnd 2nd MPU9150 Measurements\n");
	
	accel_TCBias_Slope[0] = (accel_Bias2[0] - accel_Bias1[0]) / (Temperature2 - Temperature1); //X axis
	accel_TCBias_Slope[1] = (accel_Bias2[1] - accel_Bias1[1]) / (Temperature2 - Temperature1); //Y axis
	accel_TCBias_Slope[2] = (accel_Bias2[2] - accel_Bias1[2]) / (Temperature2 - Temperature1); //Z axis
	
	accel_TCBias_Intercept[0] = accel_Bias2[0] - accel_TCBias_Slope[0]*Temperature2;
	accel_TCBias_Intercept[1] = accel_Bias2[1] - accel_TCBias_Slope[1]*Temperature2;
	accel_TCBias_Intercept[2] = accel_Bias2[2] + 8192.0f - accel_TCBias_Slope[2]*Temperature2;
	
	gyro_TCBias_Slope[0] = (gyro_Bias2[0] - gyro_Bias1[0]) / (Temperature2 - Temperature1); //ROLL
	gyro_TCBias_Slope[1] = (gyro_Bias2[1] - gyro_Bias1[1]) / (Temperature2 - Temperature1); //PITCH
	gyro_TCBias_Slope[2] = (gyro_Bias2[2] - gyro_Bias1[2]) / (Temperature2 - Temperature1); //YAW
	
	gyro_TCBias_Intercept[0] = gyro_Bias2[0] - gyro_TCBias_Slope[0]*Temperature2;
	gyro_TCBias_Intercept[1] = gyro_Bias2[1] - gyro_TCBias_Slope[1]*Temperature2;
	gyro_TCBias_Intercept[2] = gyro_Bias2[2] - gyro_TCBias_Slope[2]*Temperature2;
	
	UART_OutString((char *)"MPU9150 Calibration Complete.\n\n");
	
	UART_OutString((char *)"Accel_TCBias_Slope:\nx: ");
	UART_OutFloat(accel_TCBias_Slope[0]);
	UART_OutString((char *)"\ny: ");
	UART_OutFloat(accel_TCBias_Slope[1]);	
	UART_OutString((char *)"\nz: ");
	UART_OutFloat(accel_TCBias_Slope[2]);
	UART_OutString((char *)"\n\r");
	
	UART_OutString((char *)"Accel_TCBias_Intercept:\nx: ");
	UART_OutFloat(accel_TCBias_Intercept[0]);
	UART_OutString((char *)"\ny: ");
	UART_OutFloat(accel_TCBias_Intercept[1]);	
	UART_OutString((char *)"\nz: ");
	UART_OutFloat(accel_TCBias_Intercept[2]);
	UART_OutString((char *)"\n\r");
	
	UART_OutString((char *)"Gyro_TCBias_Slope:\nRoll: ");
	UART_OutFloat(gyro_TCBias_Slope[0]);
	UART_OutString((char *)"\nPitch: ");
	UART_OutFloat(gyro_TCBias_Slope[1]);	
	UART_OutString((char *)"\nYaw: ");
	UART_OutFloat(gyro_TCBias_Slope[2]);
	UART_OutString((char *)"\n\r");
	
	UART_OutString((char *)"Gyro_TCBias_Intercept:\nRoll: ");
	UART_OutFloat(gyro_TCBias_Intercept[0]);
	UART_OutString((char *)"\nPitch: ");
	UART_OutFloat(gyro_TCBias_Intercept[1]);	
	UART_OutString((char *)"\nYaw: ");
	UART_OutFloat(gyro_TCBias_Intercept[2]);
	UART_OutString((char *)"\n\r");
}

void MPU9150_accel_Calibration(float * Accel_AVR, float * MPU_Temp){
	int16_t index;
	float noseUp = 0.0f;
	float noseDown = 0.0f;
	float leftWingDown = 0.0f;
	float rightWingDown = 0.0f;
	float upSideDown = 0.0f;
	float rightSideUp = 0.0f;
	
	UART_OutString((char *)"\nMPU9150 Accelerometer Calibration:\n");
	
	UART_OutString((char *)"\n--> Right Side Up...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	for(index = 0; index < 5000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		rightSideUp += Accel_AVR[2] - accel_TCBias[2];
		Delay_ms(4);
	}
	rightSideUp /= 5000.0f;
	
	UART_OutString((char *)"right Side Up = ");
	UART_OutFloat(rightSideUp);
	
	UART_OutString((char *)"\n--> Up Side Down...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	for(index = 0; index < 5000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		upSideDown += Accel_AVR[2] - accel_TCBias[2];
		Delay_ms(4);
	}
	upSideDown /= 5000.0f;
	
	UART_OutString((char *)"up side down = ");
	UART_OutFloat(upSideDown);
	
	UART_OutString((char *)"\n--> Left Edge Down...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	for(index = 0; index < 5000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		leftWingDown += Accel_AVR[1] - accel_TCBias[1];
		Delay_ms(4);
	}
	leftWingDown /= 5000.0f;
	
	UART_OutString((char *)"left wing down = ");
	UART_OutFloat(leftWingDown);
	
	UART_OutString((char *)"\n--> Right Edge Down...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	for(index = 0; index < 5000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		rightWingDown += Accel_AVR[1] - accel_TCBias[1];
		Delay_ms(4);
	}
	rightWingDown /= 5000.0f;
	
	UART_OutString((char *)"right wing down = ");
	UART_OutFloat(rightWingDown);
	
	UART_OutString((char *)"\n--> Rear Edge Down...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	for(index = 0; index < 5000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		noseUp += Accel_AVR[0] - accel_TCBias[0];
		Delay_ms(4);
	}
	noseUp /= 5000.0f;
	
	UART_OutString((char *)"nose up = ");
	UART_OutFloat(noseUp);
	
	UART_OutString((char *)"\n--> Front Edge Down...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	for(index = 0; index < 5000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		noseDown += Accel_AVR[0] - accel_TCBias[0];
		Delay_ms(4);
	}
	noseDown /= 5000.0f;
	
	UART_OutString((char *)"nose down = ");
	UART_OutFloat(noseDown);
	
	accel_BiasMPU[2] = (rightSideUp + upSideDown)/2.0f;
	accel_ScaleFactorMPU[2] = (2.0f * 9.8065f) / fabsf(rightSideUp - upSideDown);
	
	accel_BiasMPU[1] = (leftWingDown + rightWingDown)/2.0f;
	accel_ScaleFactorMPU[1] = (2.0f * 9.8065f) / fabsf(leftWingDown - rightWingDown);
	
	accel_BiasMPU[0] = (noseUp + noseDown)/2.0f;
	accel_ScaleFactorMPU[0] = (2.0f * 9.8065f) / fabsf(noseUp - noseDown);
	
	UART_OutString((char *)"\nAccelerometer Calibration Complete:");
	UART_OutString((char *)"\nAccel Bias MPU: ");
	UART_OutFloat(accel_BiasMPU[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_BiasMPU[1]);	
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_BiasMPU[2]);
	UART_OutString((char *)"\nAccel Scale Factor MPU: ");
	UART_OutFloat(accel_ScaleFactorMPU[0]*1000000.0f);
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_ScaleFactorMPU[1]*1000000.0f);	
	UART_OutString((char *)" : ");
	UART_OutFloat(accel_ScaleFactorMPU[2]*1000000.0f);
	UART_OutString((char *)"\n\r");
}

// Magnetometer Calib
void MPU9150_Magnetometer_Calibration(float * Mag_AVR){
	int16_t index;
	float MagData[3];
	
	UART_OutString((char *)"\nMPU9150 Magnetometer Calibration:\n");
	
	UART_OutString((char *)"\n--> X+ point 0...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"X+ point 0: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> X+ point 180...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"X+ point 180: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> X- point 0...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"X- point 0: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> X- point 180...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"X- point 180: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Y+ point 0...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Y+ point 0: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Y+ point 180...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Y+ point 180: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Y- point 0...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Y- point 0: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Y- point 180...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Y- point 180: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Z+ point 0...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Z+ point 0: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Z+ point 180...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Z+ point 180: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Z- point 0...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Z- point 0: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
	
	UART_OutString((char *)"\n--> Z- point 180...\n");
	UART_InChar();
	UART_OutString((char *)"Gathering Data...\n");
	MagData[0] = MagData[1] = MagData[2] = 0.0f;
	for(index = 0; index < 2000; index++){
		MagData[0] += Mag_AVR[0];
		MagData[1] += Mag_AVR[1];
		MagData[2] += Mag_AVR[2];
		Delay_ms(100);
	}
	MagData[0] /= 2000.0f;
	MagData[1] /= 2000.0f;
	MagData[2] /= 2000.0f;
	
	UART_OutString((char *)"Z- point 180: ");
	UART_OutFloat(MagData[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(MagData[2]);
	UART_OutString((char *)"\n");
}

// Compute MPU9150 Temperature Compensation Bias
void MPU9150_Compute_TCBias(float * MPU_Temperature){
	accel_TCBias[0] = accel_TCBias_Slope[0] * (*MPU_Temperature) + accel_TCBias_Intercept[0];
	accel_TCBias[1] = accel_TCBias_Slope[1] * (*MPU_Temperature) + accel_TCBias_Intercept[1];
	accel_TCBias[2] = accel_TCBias_Slope[2] * (*MPU_Temperature) + accel_TCBias_Intercept[2];
	
	gyro_TCBias[0] = gyro_TCBias_Slope[0] * (*MPU_Temperature) + gyro_TCBias_Intercept[0];
	gyro_TCBias[1] = gyro_TCBias_Slope[1] * (*MPU_Temperature) + gyro_TCBias_Intercept[1];
	gyro_TCBias[2] = gyro_TCBias_Slope[2] * (*MPU_Temperature) + gyro_TCBias_Intercept[2];
}

// Compute MPU9150 Runtime Data
void computeMPU9150_RTData(float * Accel_Data, float * Gyro_Data, float * Mag_Data, float * MPU_Temp){
	int16_t index;
	float accelSum[3] = {0.0,0.0,0.0};
	float gyroSum[3] = {0.0,0.0,0.0};
	float magSum[3] = {0.0,0.0,0.0};
	
	MPU9150_calibrating = true;
	for(index = 0; index < 2000; index++){
		MPU9150_Compute_TCBias(MPU_Temp);
		accelSum[0] += (Accel_Data[0] - accel_BiasMPU[0] - accel_TCBias[0]) * accel_ScaleFactorMPU[0];
		accelSum[1] += (Accel_Data[1] - accel_BiasMPU[1] - accel_TCBias[1]) * accel_ScaleFactorMPU[1];
		accelSum[2] += (Accel_Data[2] - accel_BiasMPU[2] - accel_TCBias[2]) * accel_ScaleFactorMPU[2];
		gyroSum[0] += (Gyro_Data[0] - gyro_TCBias[0]);
		gyroSum[1] += (Gyro_Data[1] - gyro_TCBias[1]);
		gyroSum[2] += (Gyro_Data[2] - gyro_TCBias[2]);
		
		if(index % 50 == 49){
			Mag_Transformation(Mag_Data);
			Mag_vector_length_stabilasation();
			magSum[0] += Mag_calibrated[0] * 10.0f * 1229.0f / 4096.0f;
			magSum[1] += Mag_calibrated[1] * 10.0f * 1229.0f / 4096.0f;
			magSum[2] += Mag_calibrated[2] * 10.0f * 1229.0f / 4096.0f;
		}
		
		Delay_ms(2);
	}
	for(index = 0; index < 3; index++){
		accelSum[index] = accelSum[index] / 2000.0f;
		gyro_RTBias[index] = gyroSum[index] / 2000.0f;
		magSum[index] = magSum[index] / 40.0f;
	}
	
	accel_OneG = sqrtf(accelSum[0]*accelSum[0] + accelSum[1]*accelSum[1] + accelSum[2]*accelSum[2]);
	
	UART_OutString((char *)"\nAccel One G = ");
	UART_OutFloat(accel_OneG);
	UART_OutString((char *)"\nGyro RT Bias: ");
	UART_OutFloat(gyro_RTBias[0]);
	UART_OutString((char *)" : ");
	UART_OutFloat(gyro_RTBias[1]);
	UART_OutString((char *)" : ");
	UART_OutFloat(gyro_RTBias[2]);
	UART_OutString((char *)"\n\r");
	
	MPU9150_calibrating = false;
}

// Compute MPU9150 Runtime Data* (moi lan dua vao 1 du lieu)
void computeMPU9150_RuntimeData(float * Accel_Data, float * Gyro_Data, float * MPU_Temp){
		MPU9150_Compute_TCBias(MPU_Temp);
		accelSum[0] += (Accel_Data[0] - accel_BiasMPU[0] - accel_TCBias[0]) * accel_ScaleFactorMPU[0];
		accelSum[1] += (Accel_Data[1] - accel_BiasMPU[1] - accel_TCBias[1]) * accel_ScaleFactorMPU[1];
		accelSum[2] += (Accel_Data[2] - accel_BiasMPU[2] - accel_TCBias[2]) * accel_ScaleFactorMPU[2];
		gyroSum[0] += (Gyro_Data[0] - gyro_TCBias[0]);
		gyroSum[1] += (Gyro_Data[1] - gyro_TCBias[1]);
		gyroSum[2] += (Gyro_Data[2] - gyro_TCBias[2]);
}
