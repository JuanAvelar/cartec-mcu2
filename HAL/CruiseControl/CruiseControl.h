/*
^	 * CruiseControl.h
 *
 *  Created on: 14/05/2018
 *      Author: Farid
 */

#ifndef CRUISECONTROL_CRUISECONTROL_H_
#define CRUISECONTROL_CRUISECONTROL_H_

#include "GPIO.h"
#include "FTM.h"
#include "OBD2.h"


#define THROTTLE_LIMIT 60
#define ALLOWED_ERROR 10
#define MAX_setpoint 100
#define MIN_setpoint 0
#define PID_RESET_THRESHOLD_THROTTLE 2

float tps_temp;

void cruisecontrol_init(void);
void cruisecontrol_release(void);
void cruisecontrol_set_position(uint8_t tps, uint8_t set_point);
void cruisecontrol_handler(uint8_t set_point);
void cruisecontrol_handler_with_ADC(float set_point, uint8_t state_machine);

void cruisecontrol_dummy(void);
void cruisecontrol_dummy_2(uint8_t set);

#endif /* CRUISECONTROL_CRUISECONTROL_H_ */
