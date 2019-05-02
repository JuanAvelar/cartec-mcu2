/*
 * brake.c
 *
 *  Created on: 29/04/2018
 *      Author: Andres
 */



#include "Brake.h"

float check_setpoint;
FTM_QuadDec_config_t braking_encoder = {
		/* Encoder channel A */
		.port[0].port	= ePortA,
		.port[0].pin	= 0,
		.port[0].mux 	= eMux5,
		.port[0].dir	= eInput,
		/* Encoder channel B */
		.port[1].port	= ePortA,
		.port[1].pin	= 12,
		.port[1].mux 	= eMux6,
		.port[1].dir	= eInput,
		/* FTM module configuration */
		.FTM_config.FTM_instance 	 = FTM2,
		.FTM_config.ip_index 		 = PCC_FTM2_INDEX,
		.FTM_config.FTM_clock_source = SPLLDIV1,
		.Ovf_IRQn	= FTM2_Ovf_Reload_IRQn,
		.mod		= 1800,							/* (óptico)Ticks per revolution */
		.quadmode 	= phaseA_phaseB
};

FTM_PWM_config_t channel_1_PWM = {
		.FTM_config.FTM_instance 	 = FTM5,
		.FTM_config.ip_index 		 = PCC_FTM5_INDEX,
		.FTM_config.FTM_clock_source = SOSCDIV1,	/* 8 MHz SOSCDIV1_CLK */

		.preescaler = PS_1,			/* (8MHz)/1 = 8MHz */
		.channels	= 0b100000,		/* Channel 5 */
		.mod		= 400			/* 20KHz PWM period */
};
/*
FTM_PWM_config_t channel_2_PWM = {
		.FTM_config.FTM_instance 	 = FTM4,
		.FTM_config.ip_index 		 = PCC_FTM4_INDEX,
		.FTM_config.FTM_clock_source = SOSCDIV1,	// 8 MHz SOSCDIV1_CLK

		.preescaler = PS_1,			// (8MHz)/1 = 8MHz
		.channels	= 0b1000000,	// Channel 6
		.mod		= 400			// 20KHz PWM period
};
*/
PWM_channel M1_PWM = {
		.FTM_instance = FTM5,
		.number	  = 5
};
/*
PWM_channel M2_PWM = {
		.FTM_instance = FTM4,
		.number	  = 6
};
*/
arm_pid_instance_f32 braking_pid = {
	.Kp = 0.225, 	//0.125
	.Ki = 0.0722, 	//0.0522,
	.Kd = 0.008654 	//0.00654,

};

typedef enum{
	CW,
	CCW,
	Stop,
	Coast
}brake_direction;

#define pot_vs_pwmduty_relation  (2000/channel_1_PWM.mod)
int32_t brake_count = 0;
uint8_t brake_pid_reset_flag = 0x00;
void count_revolutions_brake(void);
void set_direction_brake(brake_direction dir);
void brake_limit(float *angle);

void brake_init (void){
	vnh5019_channel_1_init();
//	ADC_init();	//12bit resolution
	FTM_QD_mode_Init(braking_encoder, count_revolutions_brake);
	FTM_PWM_mode_Init(channel_1_PWM);
	arm_pid_init_f32(&braking_pid, 1);
}

float braking_encoder_read_deg(void){
	float32_t temp = (float32_t) braking_encoder.FTM_config.FTM_instance->CNT;
	temp /= (float32_t) braking_encoder.mod;
	temp += (float32_t) brake_count;
	return (-temp*360); //(-) sign to keep standard of rotation
}

void braking_manual_ctrl(void){
	uint16_t value = utility_potentiometer_position();

	uint16_t pwm_duty = 0;

	if(value <= 2000){
		pwm_duty = (2000 - value) / pot_vs_pwmduty_relation; //e.g. (0-2000)/(-5) = 400, (2000-2000)/(-5) = 0
		set_direction_brake(CW);
	}
	else if((2000 < value) && (value < 3000)){
		GPIO_clearPin(M1_INA);
		GPIO_clearPin(M1_INB);
	}
	else if(value >= 3000){
		pwm_duty = (value - 3000)/pot_vs_pwmduty_relation;
		set_direction_brake(CCW);
	}
	PWM_set_duty(M1_PWM, pwm_duty);
}

void brake_set_position_manual_ctrl(void){

	float set_point = (float)utility_potentiometer_position() - 2500;
	check_setpoint = set_point;
	float err;
	float out;

	set_point = set_point * MOTOR_WHEELS_RELATION2;

	brake_limit(&set_point);

	err = set_point - braking_encoder_read_deg();

	if( ((-PID_RESET_THRESHOLD < err) && (err < PID_RESET_THRESHOLD)) && ~brake_pid_reset_flag ){
		arm_pid_reset_f32(&braking_pid);
		brake_pid_reset_flag = 0xFF;
	}
	else if( ((err < -PID_RESET_THRESHOLD) || (PID_RESET_THRESHOLD < err)) && brake_pid_reset_flag ){
		brake_pid_reset_flag = 0x00;
	}

	out = arm_pid_f32(&braking_pid, err);

	if(out < 0){
		set_direction_brake(CW);
		out *= -1;
	}
	else{
		set_direction_brake(CCW);
	}
	if(out > 400)
		out = 400;

	PWM_set_duty(M1_PWM, out);
}

void brake_set_position(float set_point){
	float err;
	float out;

	set_point = set_point * MOTOR_WHEELS_RELATION2;

	brake_limit(&set_point);

	err = set_point - braking_encoder_read_deg();

	if( ((-PID_RESET_THRESHOLD < err) && (err < PID_RESET_THRESHOLD)) && ~brake_pid_reset_flag ){
		arm_pid_reset_f32(&braking_pid);
		brake_pid_reset_flag = 0xFF;
	}
	else if( ((err < -PID_RESET_THRESHOLD) || (PID_RESET_THRESHOLD < err)) && brake_pid_reset_flag ){
		brake_pid_reset_flag = 0x00;
	}

	out = arm_pid_f32(&braking_pid, err);

	if(out < 0){
		set_direction_brake(CW);
		out *= -1;
	}
	else{
		set_direction_brake(CCW);
	}
	if(out > 400)
		out = 400;

	PWM_set_duty(M1_PWM, out);
}

void count_revolutions_brake(void){
	if (braking_encoder.FTM_config.FTM_instance->QDCTRL & FTM_QDCTRL_TOFDIR_MASK) {
		brake_count++;
	}
	else {
		brake_count--;
	}
}

void set_direction_brake(brake_direction dir){
	if(dir == CW){
		GPIO_setPin(M1_EN);
		GPIO_setPin(M1_INA);
		GPIO_clearPin(M1_INB);
	}
	else if (dir == CCW){
		GPIO_setPin(M1_EN);
		GPIO_clearPin(M1_INA);
		GPIO_setPin(M1_INB);
	}
	else if (dir == Stop){
		GPIO_clearPin(M1_INA);
		GPIO_clearPin(M1_INB);
	}
	else if (dir == Coast){
		GPIO_clearPin(M1_EN);
	}
}

void brake_limit (float *angle){
	if (*angle > MAX_CCW2){
		*angle = MAX_CCW2;
	}
	else if (*angle < MAX_CW2) {
		*angle = MAX_CW2;
	}
}
