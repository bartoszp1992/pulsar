/*
 * pid.c
 *
 *  Created on: Feb 12, 2024
 *      Author: bartosz
 */

#include "pid.h"

void PID_init(PID_TypeDef *pid, uint32_t gainP, uint32_t gainI, uint32_t gainD,
		int32_t maxPower, int32_t minPower) {

	pid->gainD = gainD;
	pid->gainI = gainI;
	pid->gainP = gainP;

	pid->regD = 0;
	pid->regI = 0;
	pid->regP = 0;

	pid->integralSum = 0;

	pid->maxOutput = maxPower;
	pid->minOutput = minPower;
	pid->currentValue = 0;
	pid->tagetValue = 0;

	pid->output = 0;

	pid->lastError = 0;
	pid->error = 0;

}

void PID_setTarget(PID_TypeDef *pid, int32_t setValue) {
	pid->tagetValue = setValue;
}

int32_t PID_sample(PID_TypeDef *pid, int32_t readedValue) {

	pid->currentValue = readedValue;

	pid->error = pid->tagetValue - pid->currentValue;

	//>PROPORTIONAL module
	pid->regP = (int32_t) ((float) pid->error * ((float) pid->gainP / 1000));

	//INTEGRATING module
	if (pid->gainI > 0) {
		if ((pid->error > 0 && pid->output < pid->maxOutput)
				|| (pid->error < 0 && pid->output > pid->minOutput)) { //anti wind-up
			pid->integralSum += pid->error;
		}

		pid->regI = (int32_t) ((float) pid->integralSum
				* ((float) pid->gainI / 1000));
	}else{
		pid->regI = 0;
	}

	//DERIVATIVE module
	pid->regD = (int32_t) ((float) (pid->error - pid->lastError)
			* ((float) pid->gainD / 1000));

	pid->lastError = pid->error; // store for next sample

	//sum
	pid->output = pid->regP + pid->regI + pid->regD;

	if (pid->output > pid->maxOutput)
		pid->output = pid->maxOutput;
	else if (pid->output < pid->minOutput)
		pid->output = pid->minOutput;

	return pid->output;

}
