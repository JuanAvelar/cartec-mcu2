/*
 * utilities.h
 *
 *  Created on: 14/05/2018
 *      Author: Farid
 */

#ifndef UTILITIES_UTILITIES_H_
#define UTILITIES_UTILITIES_H_

#include "GPIO.h"
#include "ADC.h"
#define BENCH_TOOLS	/* Comment line if bench tools are not to be used */

extern PORT_config_t LED_RED;
extern PORT_config_t LED_GREEN;
extern PORT_config_t LED_BLUE;
extern PORT_config_t SW3;
extern PORT_config_t SW4;

void utilities_init(void);
uint32_t utility_potentiometer_position(void);
uint32_t utility_external_read_ptc15_TPS(void);
void delayPDB1(float seconds, uint8_t state);
uint8_t check_delay_flag();
void PDB1_IRQHandler(void);

/*STATE VARIABLES*/
typedef enum {
	Jetson_connected = 0,
	throttle_pot_PID_ctrl,
	brake_pot_PID_ctrl,
	steering_pot_PID_ctrl
}machine_state_t;


#ifdef BENCH_TOOLS		/* Miscellany functions for bench testing */

void delay(float ms);	/* Polling delay, maximum 1000ms */
void stopwatch(void);

#endif

#endif /* UTILITIES_UTILITIES_H_ */
