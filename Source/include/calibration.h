#include <stdbool.h>
#include <stdint.h>
#include <math.h>

extern bool MPU9150_calibrating;
extern float accelSum[3];
extern float gyroSum[3];
extern float accel_OneG;

extern float Mag_calibrated[3];
extern float accel_TCBias[3], gyro_TCBias[3];
extern float accel_BiasMPU[3];
extern float accel_ScaleFactorMPU[3];
extern float gyro_RTBias[3];
extern float accel_TCBias_Slope[3], accel_TCBias_Intercept[3];
extern float gyro_TCBias_Slope[3], gyro_TCBias_Intercept[3];

////calibration_matrix[3][3] is the transformation matrix
//	const float calibration_matrix[3][3] = 
//  {
//    {22.609f, 0.823f, -0.526f},
//    {-3.165f, 8.765f, -0.457f},
//    {2.917f, -0.111f, 5.642f}  
//  };
////bias[3] is the bias
//  const float bias[3] = 
//  {
//    -28.83f,
//    43.009f,
//    45.062f
//  }; 	

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

	
#ifdef __cplusplus
extern "C" {
#endif
	
	void Mag_Transformation(float * uncalibrated_values);
	void Mag_vector_length_stabilasation(void);
	void MPU9150_TCBias_Measurement(float * accel_raw, float * gyro_raw, int16_t * temp_raw);
	void MPU9150_Compute_TCBias(float * MPU_Temperature);
	void MPU9150_accel_Calibration(float * Accel_AVR, float * MPU_Temp);
	void MPU9150_Magnetometer_Calibration(float * Mag_AVR);
	void computeMPU9150_RTData(float * Accel_Data, float * Gyro_Data, float * Mag_Data, float * MPU_Temp);
	void computeMPU9150_RuntimeData(float * Accel_Data, float * Gyro_Data, float * MPU_Temp);

#ifdef __cplusplus
}
#endif
	
/* Sensor Calibrating: 27/10/2015  lan 1
Battery Percent: 31%
accel x-axis self test: 0.653% of factory value
accel y-axis self test: 0.532% of factory value
accel z-axis self test: 0.786% of factory value
gyro x-axis self test: 0.252% of factory value
gyro y-axis self test: -0.252% of factory value
gyro z-axis self test: 0.261% of factory value
x gyro bias = -1.725
y gyro bias = 1.366
z gyro bias = 1.190
x accel bias = -0.038
y accel bias = -0.005
z accel bias = -0.010
x magneto calibration = 1.187
y magneto calibration = 1.199
z magneto calibration = 1.238
MPU9150 is ready for active data mode...

MPU910 Calibration:
Begin 1st MPU9150 Measurements...

Temperature Reading: 31.541
Accel Bias: 69.284 : 19.412 : 16560.365
Gyro Bias: -10.812 : -2.939 : -0.599
End 1st MPU9150 Measurements
Waiting for 10 minutes for MPU9150 temp to rise...
Begin 2nd MPU9150 Measurements...

Temperature Reading: 33.983
Accel Bias: 47.764 : 9.747 : 16523.086
Gyro Bias: -8.882 : -7.168 : -0.494
End 2nd MPU9150 Measurements
MPU9150 Calibration Complete.

Accel_TCBias_Slope:
x: -8.812
y: -3.957
z: -15.266
Accel_TCBias_Intercept:
x: 347.249
y: 144.248
z: 17041.886
Gyro_TCBias_Slope:
Roll: 0.790
Pitch: -1.731
Yaw: 0.043
Gyro_TCBias_Intercept:
Roll: -35.746
Pitch: 51.684
Yaw: -1.966
*/

/* lan 2
Battery Percent: 30%
accel x-axis self test: 0.653% of factory value
accel y-axis self test: 0.598% of factory value
accel z-axis self test: 0.786% of factory value
gyro x-axis self test: 0.252% of factory value
gyro y-axis self test: 0.000% of factory value
gyro z-axis self test: 0.261% of factory value
x gyro bias = -1.748
y gyro bias = 1.358
z gyro bias = 1.152
x accel bias = -0.039
y accel bias = -0.005
z accel bias = -0.008
x magneto calibration = 1.187
y magneto calibration = 1.199
z magneto calibration = 1.238
MPU9150 is ready for active data mode...

MPU910 Calibration:
Begin 1st MPU9150 Measurements...

Temperature Reading: 31.781
Accel Bias: 93.038 : 16.296 : 8329.545
Gyro Bias: -4.959 : -2.937 : 6.490
End 1st MPU9150 Measurements
Waiting for 15 minutes for MPU9150 temp to rise...
Begin 2nd MPU9150 Measurements...

Temperature Reading: 33.777
Accel Bias: 72.613 : 7.584 : 8296.649
Gyro Bias: -1.926 : -5.523 : 7.313
End 2nd MPU9150 Measurements
MPU9150 Calibration Complete.

Accel_TCBias_Slope:
x: -10.232
y: -4.364
z: -16.480
Accel_TCBias_Intercept:
x: 418.246
y: 155.024
z: 17045.314
Gyro_TCBias_Slope:
Roll: 1.519
Pitch: -1.295
Yaw: 0.412
Gyro_TCBias_Intercept:
Roll: -53.261
Pitch: 38.234
Yaw: -6.610
*/

/* lan 3: dung Accel_AVE va Gyro_AVE
Battery Percent: 29%
accel x-axis self test: 0.653% of factory value
accel y-axis self test: 0.822% of factory value
accel z-axis self test: 0.786% of factory value
gyro x-axis self test: 0.252% of factory value
gyro y-axis self test: -0.248% of factory value
gyro z-axis self test: 0.261% of factory value
x gyro bias = -1.740
y gyro bias = 1.335
z gyro bias = 1.198
x accel bias = -0.037
y accel bias = -0.002
z accel bias = -0.008
x magneto calibration = 1.187
y magneto calibration = 1.199
z magneto calibration = 1.238
MPU9150 is ready for active data mode...

MPU910 Calibration:
Begin 1st MPU9150 Measurements...

Temperature Reading: 31.498
Accel Bias: 74.768 : 6.420 : 8331.755
Gyro Bias: 1.230 : -2.607 : 5.093
End 1st MPU9150 Measurements
Waiting for 15 minutes for MPU9150 temp to rise...
Begin 2nd MPU9150 Measurements...

Temperature Reading: 33.854
Accel Bias: 49.496 : -3.572 : 8298.597
Gyro Bias: -2.398 : -2.831 : -0.816
End 2nd MPU9150 Measurements
MPU9150 Calibration Complete.

Accel_TCBias_Slope:
x: -10.724
y: -4.240
z: -14.071
Accel_TCBias_Intercept:
x: 412.586
y: 139.991
z: 16966.976
Gyro_TCBias_Slope:
Roll: -1.540
Pitch: -0.095
Yaw: -2.507
Gyro_TCBias_Intercept:
Roll: 49.749
Pitch: 0.390
Yaw: 84.081
*/

/*
MPU9150 Accelerometer Calibration:

--> Right Side Up...
 Gathering Data...
right Side Up = -36.111
--> Up Side Down...
 Gathering Data...
up side down = -32558.988
--> Left Edge Down...
 Gathering Data...
left wing down = -16478.941
--> Right Edge Down...
 Gathering Data...
right wing down = 16309.514
--> Rear Edge Down...
 Gathering Data...
nose up = 16445.930
--> Front Edge Down...
 Gathering Data...
nose down = -16196.200
Accelerometer Calibration Complete:
Accel Bias MPU: 124.864 : -84.713 : -16297.550
Accel Scale Factor MPU: 600.849 : 598.168 : 603.052
*/

/* MPU9150 Magnetometer Calibration:

--> X+ point 0...
 Gathering Data...
X+ point 0: 41.715 : 79.354 : -114.874

--> X+ point 180...
 Gathering Data...
X+ point 180: 39.066 : -156.645 : -7.910

--> X- point 0...
 Gathering Data...
X- point 0: 60.890 : 80.347 : -8.308

--> X- point 180...
 Gathering Data...
X- point 180: 59.601 : -156.260 : -113.924

--> Y+ point 0...
 Gathering Data...
Y+ point 0: 168.632 : -49.301 : -6.367

--> Y+ point 180...
 Gathering Data...
Y+ point 180: -67.748 : -48.157 : -115.802

--> Y- point 0...
 Gathering Data...
Y- point 0: 169.271 : -28.843 : -112.184

--> Y- point 180...
 Gathering Data...
Y- point 180: -70.151 : -27.754 : -7.876

--> Z+ point 0...
 Gathering Data...
Z+ point 0: 167.152 : -91.475 : -76.408

--> Z+ point 180...
 Gathering Data...
Z+ point 180: -66.934 : 15.326 : -78.221

--> Z- point 0...
 Gathering Data...
Z- point 0: 168.024 : 12.051 : -37.623

--> Z- point 180...
 Gathering Data...
Z- point 180: -68.602 : -90.984 : -40.917
*/
