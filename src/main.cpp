/**
 *  Cartec-MCU2 JEEP proyect
 *
 */


#include "system.h" /* include peripheral declarations S32K148 */
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "ros.h"

// Include C headers (ie, non C++ headers) in this block
extern "C" {
#include "clocks_and_modes.h"
#include "utilities.h"
#include "Scheduler.h"
#include "Steering.h"
#include "Brake.h"
#include "CruiseControl.h"
#include "I2C.h"
//#include "xbox_controller.h"
}

/* Needed for AVR to use virtual functions */
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

/* Function prototypes */
void ros_callback_ctrl_pos(const std_msgs::Float32MultiArray &msg);
void ros_callback_ctrl_vel(const std_msgs::Float32MultiArray &msg);
void cruise(void);
void brake (void);
void steering(void);
void noderos(void);
void hear_i2c_and_update(void);
void check_states(void);

ros::NodeHandle* point_to_node;
ros::Publisher pub("", 0);

/* HIGH LEVEL CONTROL SIGNALS */
typedef enum {
	position,
	velocity
}control_mode_t;

struct u_signals_t {
	control_mode_t control_mode;
	float steering;
	float braking;
	float throttle;
	float vel_steering;
	float vel_braking;
	float vel_throttle;
};

struct u_signals_t u_signals;

uint8_t state = Jetson_connected;


float pos = 0;


#define NUMBER_OF_TASKS 5

scheduler_task_config_t tasks[NUMBER_OF_TASKS] = {
/*
		{
				.task_callback = noderos,
				.period_ticks  = 0x02,
				.start_tick	   = 0x01
		},
*/
		{
				.task_callback = steering,
				.period_ticks  = 2858,		// 2858*3.5us = 10.003ms
				.start_tick	   = 0x06
		},
		{
				.task_callback = brake,
				.period_ticks  = 2858,		// 2858*3.5us = 10.003ms
				.start_tick	   = 0x04
		},
		{
				.task_callback = hear_i2c_and_update,
				.period_ticks  = 2858,		// 2858*3.5us = 10.003ms
				.start_tick	   = 0x02
		},
		{
				.task_callback = cruise,
				.period_ticks  = 2858,		// 28571*3.5us = 99.9985ms ~100ms
				.start_tick	   = 0x08
		},
		{
				.task_callback = check_states,
				.period_ticks  = 2858,		// 28571*3.5us = 99.9985ms ~100ms
				.start_tick	   = 0x10
		}

};


int main(void)
{
	SOSC_init_8MHz();		/* And SOSCDIV1 & SOSCDIV2 =1: divide by 1 */
	SPLL_init_160MHz();		/* And SPLLDIV1 divide by 2; SPLLDIV2 divide by 4 */
	NormalRUNmode_80MHz();

	utilities_init();

	/*while(GPIO_readPin(SW3) != 1){ // Wait for start button
		GPIO_togglePin(LED_BLUE);
		delay(250);
	}*/

	GPIO_setPin(LED_BLUE);

/* ROS ==========================
	ros::NodeHandle nh;
	ros::Subscriber<std_msgs::Float32MultiArray> sub_pos("/board_connection/control_pos", &ros_callback_ctrl_pos);
//	ros::Subscriber<std_msgs::Float32MultiArray> sub_vel("/board_connection/control_vel", &ros_callback_ctrl_vel);

	// Publisher to check if the MCU is listening to pc/ros
	std_msgs::Int8 ros_speaker;
	pub.topic_ = "/mcu/active";
	pub.msg_ = &ros_speaker;

	nh.initNode();
	nh.advertise(pub);
	nh.subscribe(sub_pos);
//	nh.subscribe(sub_vel);

	point_to_node = &nh;
// End ROS ====================== */
#define Direccion_esclavo 68
	/*Si se utiliza la alimentación de 12V no
	 * se puede utilizar el modulo de LPI2C0
	 * ya que tiene pines compartidos
	 * fijos con otro componente
	 */
	Te_ordeno_que_te_inicies_esclavo1(Direccion_esclavo);
	//Te_ordeno_que_te_inicies_esclavo0(Direccion_esclavo);
	//obd2_init();
	steering_init();
	cruisecontrol_init();
	brake_init();

	u_signals.control_mode = position;
	u_signals.steering = 0;
	u_signals.braking = 0;
	u_signals.throttle = 0;
	u_signals.vel_braking = 0;
	u_signals.vel_steering = 0;
	u_signals.vel_throttle = 0;

	scheduler_init(&tasks[0], NUMBER_OF_TASKS, 140); //140 * 25ns = 3.5us

	GPIO_clearPin(LED_BLUE);

	for(;;){
	}

	return 0;
}



void ros_callback_ctrl_pos(const std_msgs::Float32MultiArray &msg) {
	u_signals.control_mode = position;
	u_signals.steering = msg.data[0];
	u_signals.braking  = msg.data[1];
	u_signals.throttle = msg.data[2];

	std_msgs::Int8 to_send;
	to_send.data = 0;
	pub.publish(&to_send);

}

void ros_callback_ctrl_vel(const std_msgs::Float32MultiArray &msg) {
	u_signals.control_mode = velocity;
	u_signals.vel_steering = msg.data[0];
	u_signals.vel_braking  = msg.data[1];
	u_signals.vel_throttle = msg.data[2];

	std_msgs::Int8 to_send;
	to_send.data = 1;
	pub.publish(&to_send);
}

void cruise (void){
	//cruisecontrol_handler(u_signals.throttle);
//	cruisecontrol_dummy_2(u_signals.throttle);
	cruisecontrol_handler_with_ADC(u_signals.throttle, state);
}

void brake (void){
	brake_handler(u_signals.braking, state);
	//braking_manual_ctrl();
	//brake_set_position_manual_ctrl();
}

void steering(void){
//	steering_set_position(u_signals.steering);
//	pos = steering_encoder_read_deg();
	steering_handler(u_signals.steering, state);
}

void noderos(void){
	point_to_node->spinOnce();
}
void hear_i2c_and_update(void){
	float_signals_update(	&u_signals.throttle,
							&u_signals.braking,
							&u_signals.steering );
	u_signals.control_mode = position;
}
/*State machine*/
void check_states(void){
	if(!check_delay_flag()){
		if(GPIO_readPin(SW3) && (state == Jetson_connected || state == steering_pot_PID_ctrl)){
			GPIO_clearPin(LED_RED);
			GPIO_clearPin(LED_GREEN);
			GPIO_setPin(LED_BLUE);
			state = throttle_pot_PID_ctrl;
			delayPDB1(0.5, state);//4 seconds is the maximum possible delay

		}
		else if(GPIO_readPin(SW3) && state == throttle_pot_PID_ctrl){
			GPIO_clearPin(LED_BLUE);
			GPIO_setPin(LED_GREEN);
			GPIO_clearPin(LED_RED);
			state = brake_pot_PID_ctrl;
			delayPDB1(0.5, state);
		}
		else if(GPIO_readPin(SW3) && state == brake_pot_PID_ctrl){
			GPIO_setPin(LED_RED);
			GPIO_clearPin(LED_GREEN);
			GPIO_clearPin(LED_BLUE);
			state = steering_pot_PID_ctrl;
			delayPDB1(0.5, state);
		}
	}
	if(GPIO_readPin(SW4) && state != Jetson_connected){
		GPIO_clearPin(LED_BLUE);
		GPIO_clearPin(LED_GREEN);
		GPIO_clearPin(LED_RED);
		state = Jetson_connected;
	}
}
