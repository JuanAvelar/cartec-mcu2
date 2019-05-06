/*
 * utilities.c
 *
 *  Created on: 14/05/2018
 *      Author: Farid
 */

#include "utilities.h"


PORT_config_t LED_RED   = { .port = ePortE, .pin = 21, .mux = eMuxAsGPIO, .dir = eOutput };
PORT_config_t LED_GREEN = { .port = ePortE, .pin = 22, .mux = eMuxAsGPIO, .dir = eOutput };
PORT_config_t LED_BLUE  = { .port = ePortE, .pin = 23, .mux = eMuxAsGPIO, .dir = eOutput };
PORT_config_t SW3  = { .port = ePortC, .pin = 12, .mux = eMuxAsGPIO, .dir = eInput, .attrib.filter = ePassiveFilterEnabled};
PORT_config_t SW4  = { .port = ePortC, .pin = 13, .mux = eMuxAsGPIO, .dir = eInput, .attrib.filter = ePassiveFilterEnabled };

static uint8_t block_flag = 0;
static uint8_t status = 0;


void IRQ_init_utilities(int irq,int priority){
	S32_NVIC->ICPR[irq/32] = (1 << (irq % 32));
	S32_NVIC->ISER[irq/32] = (1 << (irq % 32));
	if(priority < 16)//only 16 priority levels being 1 the most important
		S32_NVIC->IP  [irq/4]  = (uint8_t)priority << (8 * (irq % 4) + 4);
}

void LPIT0_init1 (void);
void PDB1_init(void);

void utilities_init(void){
	GPIO_pinInit(LED_RED);
	GPIO_pinInit(LED_GREEN);
	GPIO_pinInit(LED_BLUE);
	GPIO_pinInit(SW3);
	GPIO_pinInit(SW4);
	ADC_init();	//12bit resolution

#ifdef BENCH_TOOLS
	LPIT0_init1();
#endif

	/* Initial state */
	GPIO_clearPin(LED_RED);
	GPIO_clearPin(LED_GREEN);
	GPIO_clearPin(LED_BLUE);
	/*Programmable delay block*/
	PDB1_init();
	IRQ_init_utilities(PDB1_IRQn,10);
}
//Reads pin from potenciometer
uint32_t utility_potentiometer_position(void){
	convertAdcChan(0b101100);		/* Convert Channel AD28 to pot on EVB */
	while(adc_complete()==0){}      /* Wait for conversion complete flag */
	return read_adc_chx();			/* Get channel's conversion results in mv (0-5000) */
}

uint32_t utility_external_read_ptc15_TPS(void){
	convertAdcChan(0b001101);		/* Convert Channel AD13 to pot on EVB */
	while(adc_complete()==0){}      /* Wait for conversion complete flag */
	return read_adc_chx();			/* Get channel's conversion results in mv (0-5000) */
}


#ifdef BENCH_TOOLS

void delay(float ms){
	  /*Channel 1*/
	  ms /=1000;
	  ms *= 40000000;
	  LPIT0->TMR[1].TVAL = (uint32_t) ms;
	  LPIT0->TMR[1].TCTRL = 0x00000001; //Enable
	  //Wait to interrupt flag to set
	  while (0 == (LPIT0->MSR & LPIT_MSR_TIF1_MASK)) {}
	  LPIT0->MSR |= LPIT_MSR_TIF1_MASK;//write 1 to clear
}

void stopwatch(void){
	  /*Channel 1*/
	  LPIT0->TMR[1].TVAL = (uint32_t) 40000000;
	  LPIT0->TMR[1].TCTRL = 0x00000001; //Enable
}

void LPIT0_init1 (void) {
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs */
  LPIT0->MCR = 0x00000001;    /* DBG_EN-0: Timer chans stop in Debug mode */

}
/*PDB1*/

void PDB1_init(void) {
  PCC->PCCn[PCC_PDB1_INDEX] |= PCC_PCCn_CGC_MASK;     /* Enable clock for PDB0 regs */

  PDB1->SC 	|= 0x00007FAD;//Los registros no se actualizen hasta que le escribas un 1 hexadecimal

  //prescaler = 128, MULT = 10
  //Resolution 128*10*0.000 000 012 5s = 0.000128s//not making sense
  //1s = 7800 counts
  PDB1->IDLY|= (uint16_t)9999;
  PDB1->MOD	|= (uint16_t)10000;//valor máximo del contador
  PDB1->SC 	|= PDB_SC_LDOK_MASK;//set delay
}

void delayPDB1(float seconds, uint8_t state){//valor no puede ser mayor a 65000
	status = state;
	seconds = seconds*15625;//counts per second
	if(seconds > 65500){seconds = 65500;}//seconds turn to counts and maximum possible value in the register is (2^16 - 1)
	PDB1->SC 	|= PDB_SC_LDOK_MASK;//set delay
	PDB1->IDLY  = (uint16_t)seconds;
	PDB1->MOD	= (uint16_t)seconds+2;//valor máximo del contador
	PDB1->SC 	|= PDB_SC_LDOK_MASK;//set delay
	PDB1->SC  	|= PDB_SC_SWTRIG_MASK;//start PDB0 counter
	block_flag = 1;
	GPIO_setPin(LED_RED);
	GPIO_setPin(LED_GREEN);
	GPIO_setPin(LED_BLUE);
}

uint8_t check_delay_flag(){
	return block_flag;
}

void PDB1_IRQHandler(void){
	block_flag = 0;
	PDB1->SC &= ~(PDB_SC_PDBIF_MASK);//clear flag
	switch (status){
	case throttle_pot_PID_ctrl:
		GPIO_clearPin(LED_GREEN);
		GPIO_clearPin(LED_RED);

		break;
	case brake_pot_PID_ctrl:
		GPIO_clearPin(LED_BLUE);
		GPIO_clearPin(LED_RED);
		break;
	case steering_pot_PID_ctrl:
		GPIO_clearPin(LED_GREEN);
		GPIO_clearPin(LED_BLUE);
		break;
	default:
		GPIO_clearPin(LED_BLUE);
		GPIO_clearPin(LED_GREEN);
		GPIO_clearPin(LED_RED);
		//nothing
	}
}

#endif

