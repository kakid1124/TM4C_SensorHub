#include <math.h>
#include "MPU9150.h"
#include "I2C.h"
#include "Timer0A.h"

MPU9150::MPU9150(){
	Ascale = AFS_2G;
	Gscale = GFS_250DPS;
	newMagData = false;
}

void MPU9150::begin(void){
	getAres();// Get accelerometer sensitivity
	getGres();// Get gyroscope sensitivity
	
	MagRate = 10;// set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
	mRes = 10.0f * 1229.0f / 4096.0f;// Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
	
	magCalibration[0] = 0;
	magCalibration[1] = 0;
	magCalibration[2] = 0;
	
	magbias[0] = -5.0f;		// User environmental x-axis correction in milliGauss
	magbias[1] = -95.0f;	// User environmental y-axis correction in milliGauss
	magbias[2] = -260.0f;	// User environmental z-axis correction in milliGauss
	
	gyroBias[0] = 0;
	gyroBias[1] = 0;
	gyroBias[2] = 0;
	
	accelBias[0] = 0;
	accelBias[1] = 0;
	accelBias[2] = 0;
	
	// parameters for 6 DoF sensor fusion calculations
GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
beta = sqrtf(3.0f / 4.0f) * GyroMeasError;  // compute beta
GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	
deltat = 0.0f;                             // integration interval for both filter schemes
lastUpdate = 0; firstUpdate = 0; Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
q[0] = 1.0f;
q[1] = 0.0f;
q[2] = 0.0f;
q[3] = 0.0f;           // vector to hold quaternion

eInt[0] = 0.0f;
eInt[1] = 0.0f;
eInt[2] = 0.0f;              // vector to hold integral error for Mahony method
	
}	

void MPU9150::init(void){

	// Initialize MPU9150 device
	// wake up device
	writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	Delay_ms(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
	
	// get stable time source
	writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	
 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
//  writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x03);
	writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x00); // Fs = 8kHz
	
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
//  writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
	writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x0F);  // 8kHz/(1+15) = 500Hz
	
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU9150_ADDRESS, GYRO_CONFIG);
  writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

 // Set accelerometer configuration
  c =  readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the TM4C as master
	//writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x22); 
   writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0xB2); // pin active low, push-pull    
   writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9150::calibrate(float * dest1, float * dest2){
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  Delay_ms(100);  
	
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9150_ADDRESS, PWR_MGMT_2, 0x00); 
	Delay_ms(200);
	
// Configure device for bias calculation
  writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9150_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9150_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9150_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	Delay_ms(20);

// Configure MPU9150 gyro and accelerometer for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9150_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9150_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9150_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 1024 bytes in MPU9150)
  Delay_ms(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9150_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9150_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];         
	}

	  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
	
  if(accel_bias[2] > 0) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(MPU9150_ADDRESS, XG_OFFS_USRH, data[0]); 
  writeByte(MPU9150_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU9150_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU9150_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU9150_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU9150_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9150_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9150_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9150_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  writeByte(MPU9150_ADDRESS, XA_OFFSET_H, data[0]);  
  writeByte(MPU9150_ADDRESS, XA_OFFSET_L_TC, data[1]);
  writeByte(MPU9150_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9150_ADDRESS, YA_OFFSET_L_TC, data[3]);
  writeByte(MPU9150_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9150_ADDRESS, ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9150::selftest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4] = {0, 0, 0, 0};
   uint8_t selfTest[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU9150_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   Delay_ms(250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU9150_ADDRESS, SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU9150_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU9150_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU9150_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0f*0.34f)*(powf( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0f*0.34f)*(powf( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0f*0.34f)*(powf( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0f*131.0f)*(powf( 1.046f , (selfTest[3] - 1.0f) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0f*131.0f)*(powf( 1.046f , (selfTest[4] - 1.0f) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0f*131.0f)*(powf( 1.046f , (selfTest[5] - 1.0f) ));             // FT[Zg] factory trim calculation

 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
	int i;
   for (i = 0; i < 6; i++) {
     destination[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
}

void MPU9150::initAK8975A(float * destination){
  uint8_t rawData[3];  // x/y/z gyro register data stored here
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
  Delay_ms(10);
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
  Delay_ms(10);
  readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f; // Return x-axis sensitivity adjustment values
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 	
}

void MPU9150::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9150_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void MPU9150::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9150_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void MPU9150::prepareMagData(void)
{
	writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x01);
}
bool MPU9150::magDataReady(void){
	if(readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01)
		return true;
	else
		return false;
}
void MPU9150::readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
//  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x01); // toggle enable data read from magnetometer, no continuous read mode!
//  Delay_ms(8);
  // Only accept a new magnetometer data read if the data ready bit is set and 
  // if there are no sensor overflow or data read errors
//  if(readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
//  }
}

int16_t MPU9150::readTempData(void)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9150_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void MPU9150::resetMPU9150(void) {
  // reset device
  writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  Delay_ms(100);
  }

void MPU9150::getGres(void){
	switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0f/32768.0f;  // deg/second
					//gRes = 1.3323124e-4f;		// rad/second
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9150::getAres(void){
	switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;		// [g] (1 g = 9.81 m/s^2)
					//aRes = 0.0005985482f; // [m/s^2] Range = +/- 2 g (16384 lsb/g)
          break;
    case AFS_4G:
          //aRes = 4.0/32768.0;
					aRes = 0.0011970964f;	// Range = +/- 4 g (8192 lsb/g)
          break;
    case AFS_8G:
          //aRes = 8.0/32768.0;
					aRes = 0.0023941928f; // Range = +/- 8 g (4096 lsb/g)
          break;
    case AFS_16G:
          //aRes = 16.0/32768.0;
					aRes = 0.0047883855f; // Range = +/- 16 g (2048 lsb/g)
          break;
  }
}

void MPU9150::writeByte(uint8_t address, uint8_t _register, uint8_t data){
	I2C3_Write_Byte(address, _register, data);
}

uint8_t MPU9150::readByte(uint8_t address, uint8_t _register){
	return I2C3_Read_Byte(address, _register);
}

void MPU9150::readBytes(uint8_t address, uint8_t _register, uint8_t length, uint8_t * destination){
	I2C3_Read_Bytes(address, _register, length, destination);
}



//////////////////////////
void MPU9150::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
}
	
