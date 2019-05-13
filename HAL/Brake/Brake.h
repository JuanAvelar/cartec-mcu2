/*
 * brake.h
 *
 *  Created on: 29/04/2018
 *      Author: Andres
 */

#ifndef BRAKE_BRAKE_H_
#define BRAKE_BRAKE_H_

#include "utilities.h"
#include "dual_vnh5019_config.h"
#include "FTM.h"
#include "ADC.h"
#include "arm_math.h"

#define PID_RESET_THRESHOLD_BRK 100
#define PID_SAMPLING_MS 10
#define MAX_CW  -1300
#define MAX_CCW 1300
#define MOTOR_WHEELS_RELATION 1300/23

#define MAX_CW2  -157//or -43% of revolution
#define MAX_CCW2 157 //or 43% of revolution
#define MOTOR_WHEELS_RELATION2 360/100



/*First motor used, no longer used*/
void brake_init(void);

/*Currently used motor*/
void brake_set_position_manual_ctrl(void);
void brake_set_position(float set_point);
float braking_encoder_read_rev(void);
void braking_manual_ctrl(void);
void brake_handler(float set_point, uint8_t state_machine);


#endif /* BRAKE_BRAKE_H_ */
