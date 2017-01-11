/*
 * IMU_QUEST.c
 *
 *  Created on: May 20, 2015
 *      Author: nguye_000
 */

#include "IMU_QUEST.h"
#include <math.h>

//----------------------//
// Variable definitions
volatile bool first = true;

static float p,q,r;
static float h1,h2,h3;
static float b1,b2,b3;
static float phi,theta,psi;
static float Xh, Yh;
static float qs[4], error[4];
static float recipNorm;
//======================//
// 			Functions
void QUEST(float* pfAccel, float* pfGyro, float* pfMagneto, float* pfEulers, float* qk, float* deltaT)
{
	p = pfGyro[0];	// [rad/s]
	q = pfGyro[1];
	r = pfGyro[2];
	
	h1 = pfAccel[0];			// [m/s^2]
	h2 = pfAccel[1];
	h3 = pfAccel[2];
	
	b1 = pfMagneto[0] * 0.01f;	// [milliGauss] * 10^(-2)
	b2 = pfMagneto[1] * 0.01f;
	b3 = pfMagneto[2] * 0.01f;
	
	recipNorm = invSqrt(b1*b1 + b2*b2 + b3*b3);
	b1 *= recipNorm;
	b2 *= recipNorm;
	b3 *= recipNorm;
	
	// From accelerometer
	recipNorm = invSqrt(h1*h1 + h2*h2 + h3*h3);
	phi = atan2f(h2, h3);
	theta = asinf(-h1 * recipNorm);

	// From magnetic sensor
	Xh = b1*cosf(theta) + b2*sinf(theta)*sinf(phi) + b3*sinf(theta)*cosf(phi);
	Yh = b2*cosf(phi) - b3*sinf(phi);

	psi = atan2f(-Yh, Xh);

	Euler_to_Quaternion((float*)qs, phi, theta, psi);

	if (first == true)
	{
		qk[0] = qs[0];
		qk[1] = qs[1];
		qk[2] = qs[2];
		qk[3] = qs[3];

		first = false;
	}

	error[0] = qs[0] - qk[0];
	error[1] = qs[1] - qk[1];
	error[2] = qs[2] - qk[2];
	error[3] = qs[3] - qk[3];

	qk[0] += 0.5f * (*deltaT) * (0*qk[0] - p*qk[1] - q*qk[2] - r*qk[3]) + delta*error[0];
	qk[1] += 0.5f * (*deltaT) * (p*qk[0] + 0*qk[1] + r*qk[2] - q*qk[3]) + delta*error[1];
	qk[2] += 0.5f * (*deltaT) * (q*qk[0] - r*qk[1] + 0*qk[2] + p*qk[3]) + delta*error[2];
	qk[3] += 0.5f * (*deltaT) * (r*qk[0] + q*qk[1] - p*qk[2] + 0*qk[3]) + delta*error[3];

	recipNorm = invSqrt(qk[0]*qk[0] + qk[1]*qk[1] + qk[2]*qk[2] + qk[3]*qk[3]);
	qk[0] *= recipNorm;
	qk[1] *= recipNorm;
	qk[2] *= recipNorm;
	qk[3] *= recipNorm;
	
	if (qk[0] < 0)
	{
		qk[0] = -qk[0];
		qk[1] = -qk[1];
		qk[2] = -qk[2];
		qk[3] = -qk[3];
	}
	
	Quaternion_To_Euler(qk, pfEulers);
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
	
//	unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
//	float tmp = *(float*)&i;
//	float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
//	return y;
}

void Euler_to_Quaternion(float* pfQOut, float phi, float theta, float psi)
{
    float fCOSY, fCOSP, fCOSR;
    float fSINY, fSINP, fSINR;
    float recipNorm;
	
    // Pre-calculate the cosine of (yaw, pitch, roll divided by 2)
    fCOSY = cosf(phi / 2.0f);
    fCOSP = cosf(theta / 2.0f);
    fCOSR = cosf(psi / 2.0f);

    // Pre-calculate the sine of (yaw, pitch, roll divided by 2)
    fSINY = sinf(phi / 2.0f);
    fSINP = sinf(theta / 2.0f);
    fSINR = sinf(psi / 2.0f);

    // The W component
    pfQOut[0] = fCOSY * fCOSP * fCOSR + fSINY * fSINP * fSINR;

    // The X component
    pfQOut[1] = fSINY * fCOSP * fCOSR - fCOSY * fSINP * fSINR;

    // The Y component
    pfQOut[2] = fCOSY * fSINP * fCOSR + fSINY * fCOSP * fSINR;

    // The Z component
    pfQOut[3] = fCOSY * fCOSP * fSINR - fSINY * fSINP * fCOSR;

    recipNorm = invSqrt(pfQOut[0]*pfQOut[0] + pfQOut[1]*pfQOut[1] + pfQOut[2]*pfQOut[2] + pfQOut[3]*pfQOut[3]);
    pfQOut[0] *= recipNorm;
    pfQOut[1] *= recipNorm;
    pfQOut[2] *= recipNorm;
    pfQOut[3] *= recipNorm;

    if (pfQOut[0] < 0)
    {
    	pfQOut[0] = -pfQOut[0];
    	pfQOut[1] = -pfQOut[1];
    	pfQOut[2] = -pfQOut[2];
    	pfQOut[3] = -pfQOut[3];
    }
}

void Quaternion_To_Euler(float* q, float* pfEulers)
{
		pfEulers[1] = asinf(2.0f * (q[0]*q[2] - q[1]*q[3]));

  if ((pfEulers[1] < POLE) && (pfEulers[1] > -POLE)){
		pfEulers[0] = atan2f(2.0f * (q[2]*q[3] + q[0]*q[1]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]));}
	
	
		pfEulers[2] = atan2f(2.0f * (q[1]*q[2] + q[0]*q[3]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])); 
}
