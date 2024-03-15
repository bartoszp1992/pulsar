/*
 * pid.h
 *
 *  Created on: Feb 12, 2024
 *      Author: bartosz
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct{

	//gains(promiles)
	float gainP;
	float gainI;
	float gainD;

	//regulatator output values
	int32_t regP;
	int32_t regI;
	int32_t regD;


	int32_t maxOutput;
	int32_t minOutput;


	int32_t currentValue;
	int32_t tagetValue;

	int32_t error;
	int32_t lastError;

	int64_t integralSum;

	//sum of output values
	int32_t output;



}PID_TypeDef;

void PID_init(PID_TypeDef *pid, uint32_t gainP, uint32_t gainI, uint32_t gainD,
		int32_t maxPower, int32_t minPower);
void PID_setTarget(PID_TypeDef* pid, int32_t setValue);
int32_t PID_sample(PID_TypeDef* pid, int32_t readedValue);

#endif /* PID_H_ */
