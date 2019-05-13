/*
 * Sliding_mode.c
 *
 *  Created on: 13/05/2019
 *      Author: Juan Avelar
 */
#include "Sliding_mode.h"
#include "stdlib.h"
#include "math.h"

typedef enum {
	negative = -1,
	zero	 = 0,
	positive = 1
} sign;

void init_STC_f32(arm_STC_instance_f32 * S, float32_t ts){
	S->sigma 	= 0;
	S->W 		= 0;
	S->u 		= 0;

	S->time_step 		= ts;//in seconds
	S->previous_error 	= 0;
}

float32_t arm_STC_f32(arm_STC_instance_f32 * S, float32_t error){
	S->sigma = S->C*error + (error - S->previous_error)/(S->time_step);
	sign signo;
	if(S->sigma > 0)
		signo = positive;
	else if(S->sigma < 0)
		signo = negative;
	else
		signo = zero;

	S->W = S->W + signo*S->B*S->time_step;
	S->u = S->C1*signo*sqrt(abs(S->sigma)) + S->W;

	S->previous_error = error;

	return S->u;
}


