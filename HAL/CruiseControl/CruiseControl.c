/*
 * CruiseControl.c
 *
 *  Created on: 14/05/2018
 *      Author: Farid
 */

#include "CruiseControl.h"
#include "arm_math.h"
#include "utilities.h"

PORT_config_t ENA = { .port = ePortC, .pin = 29, .mux = eMux2,      .dir = eOutput };
PORT_config_t IN1 = { .port = ePortC, .pin = 30, .mux = eMuxAsGPIO, .dir = eOutput };
PORT_config_t IN2 = { .port = ePortC, .pin = 31, .mux = eMuxAsGPIO, .dir = eOutput };
PORT_config_t IN4 = { .port = ePortE, .pin = 20, .mux = eMuxAsGPIO, .dir = eOutput };

FTM_PWM_config_t ENA_PWM_config = {
		.FTM_config.FTM_instance 	 = FTM5,
		.FTM_config.ip_index 		 = PCC_FTM5_INDEX,
		.FTM_config.FTM_clock_source = SOSCDIV1,	/* 8 MHz SOSCDIV1_CLK */

		.preescaler = PS_1,			/* (8MHz)/1 = 8MHz */
		.channels	= 0b100,			/* Channel 2 */
		.mod		= 400			/* 20KHz PWM period */
};

PWM_channel ENA_PWM = { .FTM_instance = FTM5, .number = 2 };

typedef enum{
	speed_down,
	speed_up,
	halt
}throttle_dir;

uint8_t obd_flag = 0;
float tps_value = 0;

arm_pid_instance_f32 throttle_pid = {
	.Kp = 4, 	//0.125
	.Ki = 0.722, 	//0.0522,
	.Kd = 0.2654 	//0.00654,

};

uint8_t throttle_pid_reset_flag = 0x00;

void set_throttle_action(throttle_dir dir);
void cruisecontrol_limit (float *angle);

void cruisecontrol_init(void){
//	ADC_init();	//12bit resolution
	GPIO_pinInit(IN1);
	GPIO_pinInit(IN2);
	GPIO_pinInit(IN4);
	PORT_init(ENA);
	FTM_PWM_mode_Init(ENA_PWM_config);
	arm_pid_init_f32(&throttle_pid, 1);
}


void cruisecontrol_release(void){
	GPIO_clearPin(IN4);
}

void cruisecontrol_dummy(void){
	if(GPIO_readPin(SW3))
		GPIO_setPin(IN4);
	else
		GPIO_clearPin(IN4);
	if(GPIO_readPin(SW4)){
		GPIO_clearPin(IN2);
		GPIO_setPin(IN1);
		PWM_set_duty(ENA_PWM, 400);
	}
	else{
		GPIO_clearPin(IN1);
		GPIO_clearPin(IN2);
		PWM_set_duty(ENA_PWM, 0);
	}

}

void cruisecontrol_dummy_2(uint8_t set){
	if(set){
		GPIO_setPin(IN4);
		GPIO_clearPin(IN2);
		GPIO_setPin(IN1);
		PWM_set_duty(ENA_PWM, 400);
	}
	else{
		GPIO_clearPin(IN4);
		GPIO_clearPin(IN1);
		GPIO_clearPin(IN2);
		PWM_set_duty(ENA_PWM, 0);
	}
}

void cruisecontrol_set_position(uint8_t tps, uint8_t set_point){
	if(set_point >= THROTTLE_LIMIT)
		set_point = THROTTLE_LIMIT;

	if(set_point <= 17){
		cruisecontrol_release();
	}
	else if((set_point - ALLOWED_ERROR) < tps && tps < (set_point + ALLOWED_ERROR)){
		set_throttle_action(halt);
		PWM_set_duty(ENA_PWM, 0);
	}
	else if(tps < set_point){
		set_throttle_action(speed_up);
		PWM_set_duty(ENA_PWM, 400);
	}
	else if(tps > set_point){
		set_throttle_action(speed_down);
		PWM_set_duty(ENA_PWM, 400);
	}

}

void cruisecontrol_set_position_forTPS_ADC(float tps, float set_point){
	float err;
	float out;
	//TPS limits are 0.79V and 3.88V
	//We must expand it to go from 0 to 100 to express it as percentage
	tps = (tps - 0.79)/(3.9-0.79)*100;

	cruisecontrol_limit(&set_point);

	err = set_point - tps;
//accounting on Actuator saturation
	if( ((-PID_RESET_THRESHOLD_THROTTLE < err) && (err < PID_RESET_THRESHOLD_THROTTLE)) && ~throttle_pid_reset_flag ){
		arm_pid_reset_f32(&throttle_pid);
		throttle_pid_reset_flag = 0xFF;
	}
	else if( ((err < -PID_RESET_THRESHOLD_THROTTLE) || (PID_RESET_THRESHOLD_THROTTLE < err)) && throttle_pid_reset_flag ){
		throttle_pid_reset_flag = 0x00;
	}

	out = arm_pid_f32(&throttle_pid, err);

	if(out < 0){
		set_throttle_action(speed_down);
		out *= -1;
	}
	else{
		set_throttle_action(speed_up);
	}
	if(out > 400)
		out = 400;
	if(set_point == 0){
		set_throttle_action(halt);
	}
	PWM_set_duty(ENA_PWM, out);
}

void cruisecontrol_set_pot_manual_ctrl(float tps){
	float err;
	float out;
	float set_point = (float)utility_potentiometer_position();
	//scale from 5000 to 100
	set_point = set_point*0.02;
	//TPS limits are 0.79V and 3.88V
	//We must expand it to go from 0 to 100 to express it as percentage
//set-point is good if managed from 5% to 15%
	tps = (tps - 0.79)/(3.9-0.79)*100;

	cruisecontrol_limit(&set_point);

	err = set_point - tps;
//accounting on Actuator saturation
	if( ((-PID_RESET_THRESHOLD_THROTTLE < err) && (err < PID_RESET_THRESHOLD_THROTTLE)) && ~throttle_pid_reset_flag ){
		//arm_pid_reset_f32(&throttle_pid);
		throttle_pid_reset_flag = 0xFF;
	}
	else if( ((err < -PID_RESET_THRESHOLD_THROTTLE) || (PID_RESET_THRESHOLD_THROTTLE < err)) && throttle_pid_reset_flag ){
		throttle_pid_reset_flag = 0x00;
	}

	out = arm_pid_f32(&throttle_pid, err);

	if(out < 0){
		set_throttle_action(speed_down);
		out *= -1;
	}
	else{
		set_throttle_action(speed_up);
	}
	//acceleration absolute PWM limit 300/400
	if(out > 300)
		out = 300;
	if(set_point == 0){
		set_throttle_action(halt);
	}

	PWM_set_duty(ENA_PWM, out);
}

void cruisecontrol_handler(uint8_t set_point){
	if(obd2_readable() == 1){//TPS = throttle position sensor
		obd2_read_PID(PID_TPS, &tps_value);
		obd2_request_PID(PID_TPS);
	}
	else if(obd_flag == 0){
		obd_flag = 1;
		obd2_request_PID(PID_TPS);
	}
	cruisecontrol_set_position(tps_value, set_point);
}
//----------------Main
void cruisecontrol_handler_with_ADC(float set_point, uint8_t state_machine){
	//Be careful not to measure values below 0V or above 5V
	tps_value = (float)utility_external_read_ptc15_TPS();//The read value in milivolts from the tps
	tps_value = tps_value*0.001;
	//change this function to involve a PID controller
	//limit values 3.9V high - 0.78V low

	if (state_machine == Jetson_connected){
		cruisecontrol_set_position_forTPS_ADC(tps_value, set_point);
	}
	if (state_machine == throttle_pot_PID_ctrl){
		cruisecontrol_set_pot_manual_ctrl(tps_value);
	}
}

/* ========================================================================================= */

void set_throttle_action(throttle_dir dir){
	GPIO_setPin(IN4);	// Clutch
	if(dir == speed_up){
		GPIO_setPin(IN1);
		GPIO_clearPin(IN2);
	}
	else if(dir == speed_down){
		GPIO_clearPin(IN1);
		GPIO_setPin(IN2);
	}
	else if(dir == halt){
		GPIO_clearPin(IN1);
		GPIO_clearPin(IN2);
	}
}

void cruisecontrol_limit (float *angle){
	if (*angle > MAX_setpoint){
		*angle = MAX_setpoint;
	}
	else if (*angle < MIN_setpoint) {
		*angle = MIN_setpoint;
	}
}
