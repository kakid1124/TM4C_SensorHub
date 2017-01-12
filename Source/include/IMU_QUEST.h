/*
 * IMU_QUEST.h
 *
 *  Created on: May 20, 2015
 *      Author: nguye_000
 */

#ifndef IMU_QUEST_H_
#define IMU_QUEST_H_

#include <stdbool.h>

//-----------------------//
// Definitions
#define delta		0.00049999f

#ifndef PI
#define PI		3.14159265358979323846f
#endif
#define twoPI	6.283185307f
#define D2R		0.01745329252f
#define R2D		57.29577951f
#define POLE	1.520796327f 	// = PI/2.0f - 0.05f fix roll near poles with this tolerance

//-----------------------//
// Function declarations
#ifdef __cplusplus
extern "C" {
#endif
	
	void QUEST(float* pfAccel, float* pfGyro, float* pfMagneto, float* pfEulers, float* qk, float* deltaT);
	void Quaternion_To_Euler(float* q, float* pfEulers);
	void Euler_to_Quaternion(float* pfQOut, float phi, float theta, float psi);
	float invSqrt(float x);
	
#ifdef __cplusplus
}
#endif

#endif /* IMU_QUEST_H_ */
