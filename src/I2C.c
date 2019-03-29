/*
 * I2C.c
 *
 *  Created on: 19/02/2019
 *      Author: Juan Avelar
 */
#include "S32K148.h"           /* include peripheral declarations S32K144 */
#include "I2C.h"
#include "PORT_config.h"
#include "utilities.h"

#define PTD15 		15 	/* RED LED*/
#define PTD0 		0   /* BLUE LED*/
#define PTD16		16	/* GREEN LED*/
#define SLAVE_FLOATS_RECEIVED 	3
#define BYTESperFLOAT 			4
#define CHANGE_ENDIANNES 		0
#define True		1
#define False		0
#define I2C0_SDA 	2
#define I2C0_SCL 	3
#define PULL_UP 	(uint8_t)3

static int slave_byte_count;
static uint8_t data[16];
//No sirven dentro de esta libreria solo para indicar los valores de los pines.
//PORT_i2c_init le escribe directo a los registros.
PORT_config_t config_port_i2c_sda = {
		.port 	= ePortA,
		.pin  	= I2C0_SDA,
		.mux	= eMux3,
		.dir	= eOutput
};
PORT_config_t config_port_i2c_scl = {
		.port 	= ePortA,
		.pin  	= I2C0_SCL,
		.mux	= eMux3,
		.dir	= eOutput
};
/*
void Rojo(void){
	PTD->PCOR |= 1<<PTD15;
}
void Verde(void){
	PTD->PCOR |= 1<<PTD16;
}
void Blue(void){
	PTD->PCOR |= 1<<PTD0;
}
*/
void PDB0_init (void) {
  PCC->PCCn[PCC_PDB0_INDEX] |= PCC_PCCn_CGC_MASK;     /* Enable clock for LPUART1 regs */
  PDB0->IDLY|= 999;
  PDB0->MOD	|= 1000;//valor máximo del contador
  PDB0->SC 	|= 0x00007FA1;//Los registros no se actualizen hasta que le escribas un 1 hexadecimal
}

void delay_PDB(int period){//valor no puede ser mayor a 65000
	PDB0->IDLY  |= period;
	PDB0->MOD	|= period+2;//valor máximo del contador
	PDB0->SC 	|= 0x00000001;//set delay
	PDB0->SC  	|= 1<<16;//start PDB0 counter
}

void PDB0_IRQHandler(void){
	PTD->PSOR |= 1 << PTD0;
	PDB0->SC &= ~(1<<6);//clear flag
}

/*******************************************************************************
Function Name : LPI2C0_IRQs_init
*******************************************************************************/
void IRQ_init(int irq,int priority){
	S32_NVIC->ICPR[irq/32] = (1 << (irq % 32));
	S32_NVIC->ISER[irq/32] = (1 << (irq % 32));
	if(priority < 16)//only 16 priority levels being 1 the most important
		S32_NVIC->IP  [irq/4]  = (uint8_t)priority << (8 * (irq % 4) + 4);
}
void LPI2C0_IRQs_init(void){
    // LPI2C_Slave_IRQHandler
    IRQ_init(LPI2C0_Slave_IRQn,1);// Priority level 1
    // LPI2C_Master_IRQHandler
    //IRQ_init(LPI2C0_Master_IRQn,1);                // Priority level 1
    // PDB0_IRQHandler
    IRQ_init(PDB0_IRQn,10);
}

void PORT_i2c_init(void){

	//I2C PTA2-SDA PTA3-SCL
	PCC->PCCn[PCC_PORTA_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTA */
//quita resistencia pull up interna
	PORTA->PCR[I2C0_SDA]&=  ~(PULL_UP);           /* Port B6: MUX = ALT2,I2C_SDA */
	PORTA->PCR[I2C0_SCL]&=	~(PULL_UP);           /* Port B7: MUX = ALT2,I2C_SCL */

	PORTA->PCR[I2C0_SDA]|=PORT_PCR_MUX(3) ;           /* Port B6: MUX = ALT2,I2C_SDA */
	PORTA->PCR[I2C0_SCL]|=PORT_PCR_MUX(3) ;           /* Port B7: MUX = ALT2,I2C_SCL */

}

void Te_ordeno_que_te_inicies_esclavo0(int direction){//Este esclavo solo recibe datos por interrupcion

	PDB0_init();
	PCC->PCCn[PCC_LPI2C0_INDEX] &= ~PCC_PCCn_CGC_MASK;    /* Ensure clk disabled for config */
	PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_PCS(0b001)    /* Clock Src= 1 (SOSCDIV2_CLK) */
	                            |  PCC_PCCn_CGC_MASK;     /* Enable clock for LPUART1 regs */
	//Esclavo no configura el clock SCL
	LPI2C0->SIER /*|= LPI2C_SIER_AM0IE(1)*/	//Address Match 0 	Interrupt Enable c
				   |=	LPI2C_SIER_AVIE(1)	//Address Valid 	Interrupt Enable c
				   |	LPI2C_SIER_RSIE(1)	//Repeated Start 	Interrupt Enable
				   |	LPI2C_SIER_SDIE(1)	//STOP Detect 		Interrupt Enable
				   |	LPI2C_SIER_BEIE(1)	//Bit Error 		Interrupt Enable c
				   |	LPI2C_SIER_FEIE(1)	//FIFO Error 		Interrupt Enable c
				   |	LPI2C_SIER_RDIE(1);	//Receive Data 		Interrupt Enable c
	LPI2C0->SCFGR1 |= 0x00000000;
	LPI2C0->SCFGR2 |= 0x0000000F;
	LPI2C0->SAMR   |= LPI2C_SAMR_ADDR0(direction);//Direccion I2C esclavo
	LPI2C0->SCR    |= LPI2C_SCR_SEN_MASK;//Prender esclavo

	//updated_data->address_written 	= 0;
	//updated_data->slave_data 		= 0;

	slave_byte_count = 0;
	LPI2C0_IRQs_init();
	PORT_i2c_init();
}



void LPI2C0_init_master(void){
	  PCC->PCCn[PCC_LPI2C0_INDEX] &= ~PCC_PCCn_CGC_MASK;    /* Ensure clk disabled for config */
	  PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_PCS(0b001)    /* Clock Src= 1 (SOSCDIV2_CLK) */
	                              |  PCC_PCCn_CGC_MASK;     /* Enable clock for LPUART1 regs */
	    LPI2C0->MCCR0 |= 0x0204050B;//200kbps
	    // [24] DATAVD  0x02
	    // [16] SETHOLD 0x04
	    // [8]  CLKHI   0x05
	    // [0]  CLKLO   0x0B

	    // Master Interrupt Enable Register (MIER)
	    LPI2C0->MIER |= 0x1F00;
	    // [14] DMIE = 0  (Data match interrupt disabled)
	    // [13] PLTIE = 1 (Pin low timeout interrupt enabled)
	    // [12] FEIE = 1  (FIFO error interrupt enabled)
	    // [11] ALIE = 1  (Arbitration lost interrupt enabled)
	    // [10] NDIE = 1  (NACK detect interrupt enabled)
	    // [9]  SDIE = 1  (STOP detect interrupt enabled)
	    // [8]  EPIE = 1  (End packet interrupt enabled)

	    // Master Configuration Register 0
	    LPI2C0->MCFGR0 |= 0x0000;
	    // [9] RDMO = 0    (Received data is stored in the receive FIFO as normal)
	    // [8] CIRFIFO = 0 (Circular FIFO is disabled)
	    // [2] HRSEL = 0   (Host request input is pin HREQ)
	    // [1] HRPOL = 0   (Active low)
	    // [0] HREN = 0    (Host request input is disabled)

	    // Master Configuration Register 1
	    LPI2C0->MCFGR1 |= 0x00000803;
	    // [26-14] PINCFG     = 0b000 (LPI2C configured for 2-pin open drain mode)
	    // [16-16] MATCFG     = 0b000 (Match disabled)
	    // [10]    TIMECFG    = 1     (Pin Low Timeout Flag will set if either SCL or SDA is low for longer than the configured timeout)
	    // [9]     IGNACK     = 0     (LPI2C Master will receive ACK and NACK normally)
	    // [8]     AUTOSTOP   = 0     (Without autostop generation)
	    // [2-0]   PRESCALE   = 0b001 (Divided by 2)

	    // Master Configuration Register 2
	    LPI2C0->MCFGR2 |= 0x000000FF;
	    // [27-24] FILTSDA = 0x0  (Disable the glitch filter)
	    // [19-16] FILTSDA = 0x0  (Disable the glitch filter)
	    // [11-0]  BUSIDLE = 0xFF (Bus idle timeout period in clock cycles)

	    // Master Configuration Register 3
	    LPI2C0->MCFGR3 |= 0x00000800;
	    // [19-8] PINLOW = 0x0008 (Pin Low Timeout enabled)

	    // Master FIFO Control Register
	    LPI2C0->MFCR |= 0x00000003;
	    // [17-16] RXWATER = 0 (Receive FIFO watermark)
	    // [1-0]   TXWATER = 0 (Transmit FIFO watermark)

	    // Master Control Register
	    LPI2C0->MCR |= 0x30D;
	    // [9] RRF = 1   (Receive FIFO is reset)
	    // [8] RTF = 1   (Transmit FIFO is reset)
	    // [3] DBGEN = 1 (Master is enabled in debug mode)
	    // [2] DOZEN = 1 (Master is enabled in Doze mode)
	    // [1] RST = 0   (Master logic is not reset)
	    // [0] MEN = 1   (Master logic is enabled)

}

/*******************************************************************************
Function Name : I2C_bus_busy
Returns       : uint8_t
Notes         : function checks if bus is idle
*******************************************************************************/
uint8_t I2C_bus_busy(void) {
    uint16_t timeout = 0;
    while ((LPI2C0->MSR & (1 << 25)) && (timeout < BUSY_TIMEOUT))  ++timeout;

    if(timeout >= BUSY_TIMEOUT) return BUSY;
    else return OK;
}
/*******************************************************************************
Function Name : I2C_start_ACK
Parameters    : uint8_t address
Notes         : function generates start condition, expects ACK
*******************************************************************************/
void I2C_start_ACK(uint8_t address)
{
	uint32_t command    = (address << 0);
    command             |= (0b100 << 8);
    LPI2C0->MTDR = command;
}
/*******************************************************************************
Function Name : I2C_start_NACK
Parameters    : uint8_t address
Notes         : function generates start condition, expects NACK
*******************************************************************************/
void I2C_start_NACK(uint8_t address)
{
    uint32_t command    = (address << 0);
    command             |= (0b101 << 8);
    LPI2C0->MTDR = command;
}
/*******************************************************************************
Function Name : I2C_stop
Notes         : function generates stop condition
*******************************************************************************/
void I2C_stop(void)
{
    LPI2C0->MTDR = 0x0200;
}
/*******************************************************************************
Function Name : I2C_write_byte
Parameters    : uint8_t data
Notes         : function writes one byte in TX FIFO
*******************************************************************************/
void I2C_write_byte(uint8_t data)
{
    LPI2C0->MTDR = data;
}
/*******************************************************************************
Function Name : I2C_write_reg
Parameters    : uint8_t address, uint8_t reg, uint8_t val
Returns       : uint8_t
Notes         : function writes to a register of a slave device
*******************************************************************************/
uint8_t I2C_write_reg(uint8_t address, uint8_t reg, uint16_t val)
{
  while(I2C_bus_busy()){
	  PTD->PCOR |= 1 << PTD0;
	  PTD->PSOR |= 1 << PTD16;
	  PTD->PSOR |= 1 << PTD15;
  }
  PTD->PSOR |= 1 << PTD0;
  I2C_start_ACK(address);
  I2C_write_byte(reg);                  // Send Register
  I2C_write_byte((uint8_t)(val >> 0  & 0xFF));                  // Send Value
  I2C_write_byte((uint8_t)(val >> 8  & 0xFF));
  I2C_stop();                           // Send Stop
  return OK;
}

uint8_t I2C_write_reg_32(uint8_t address, uint8_t reg, uint32_t val)
{
  while(I2C_bus_busy()) {
	  PTD->PCOR |= 1 << PTD0;
	  PTD->PSOR |= 1 << PTD16;
	  PTD->PSOR |= 1 << PTD15;

  }
  PTD->PSOR |= 1 << PTD0;
  I2C_start_ACK(address);
  I2C_write_byte(reg);                  // Send Register
  I2C_write_byte((uint8_t)(val >> 0  & 0xFF));                  // Send Value
  I2C_write_byte((uint8_t)(val >> 8  & 0xFF));
  I2C_write_byte((uint8_t)(val >> 16 & 0xFF));
  I2C_write_byte((uint8_t)(val >> 24 & 0xFF));
  I2C_stop();                           // Send Stop
  return OK;
}
/*******************************************************************************
Function Name : I2C_read
Parameters    : uint8_t *p_buffer, uint8_t n_bytes
Modifies      : uint8_t *p_buffer
Returns       : uint8_t
Notes         : function reads a register of a slave device
*******************************************************************************/
uint8_t I2C_read(uint8_t *p_buffer, uint8_t n_bytes)
{
  uint8_t  n;
  uint16_t time;
  //uint16_t timeout = (READING_TIMEOUT * n_bytes);
  uint16_t command;

  command =    0x0100;
  command |=  (n_bytes - 1);
  LPI2C0->MTDR = command;

  while (((LPI2C0->MFSR) >> 16 != n_bytes) /*&& (time < timeout)*/) ++time;

  /*if(time >= timeout){
	LPI2C0->MCR |= (1 << 9);     // reset receive FIFO
	return NO_DATA_RECEIVED;
  }*/
  for(n = 0; n < n_bytes; ++n)
  {
    p_buffer[n] = (uint8_t)(LPI2C0->MRDR & 0x000000FF);
  }
  return OK;
}
/*******************************************************************************
Function Name : I2C_read_reg
Parameters    : uint8_t address, uint8_t reg, uint8_t *p_buffer
Modifies      : uint8_t *p_buffer
Returns       : uint8_t
Notes         : function reads one register of a slave device
*******************************************************************************/
uint8_t I2C_read_reg(uint8_t address, uint8_t reg, uint8_t *p_buffer)
{
  if(I2C_bus_busy()) return BUSY;
  I2C_start_ACK(address);   							// Send Start
  I2C_write_byte(reg);
  I2C_start_ACK(address);       							// Send Repeated start
  if(I2C_read(p_buffer, 1)) return NO_DATA_RECEIVED;        // Read one register
  I2C_stop();                   							// Send Stop
  return OK;
}
/*******************************************************************************
Function Name : I2C_read_multiple_reg
Parameters    : uint8_t address, uint8_t reg, uint8_t *p_buffer, uint8_t n_bytes
Modifies      : uint8_t *p_buffer
Returns       : uint8_t
Notes         : function reads multiple registers of a slave device
*******************************************************************************/
uint8_t I2C_read_multiple_reg(uint8_t address, uint8_t reg, uint8_t *p_buffer, uint8_t n_bytes)
{
  while(I2C_bus_busy()){
	  PTD->PCOR |= 1 << PTD0;
	  PTD->PSOR |= 1 << PTD16;
	  PTD->PSOR |= 1 << PTD15;
  }
  PTD->PSOR |= 1 << PTD0;
  I2C_start_ACK(address);                               // Send Start
  I2C_write_byte(reg);
  I2C_start_ACK(address);                                   // Send Repeated start
  if(I2C_read(p_buffer, n_bytes)) return NO_DATA_RECEIVED;  // Read registers
  I2C_stop();                                               // Send Stop
  return OK;
}



/*******************************************************************************
Function Name : LPI2C0_Master_IRQHandler
*******************************************************************************/
void LPI2C0_Master_IRQHandler(void) {

    if(LPI2C0->MSR & (1 << 8))
    {
        // Master has generated a STOP or Repeated START condition
        LPI2C0->MSR = 0x100;      // clear EPF
    }
    if(LPI2C0->MSR & (1 << 9))
    {
        // Master has generated a STOP condition.
        LPI2C0->MSR = 0x200;      // clear SDF

    }
    if(LPI2C0->MSR & (1 << 10))
    {PTD->PCOR |= 1<<PTD15;
        // NACK/ACK detected and expecting ACK/NACK for address byte
        // When set, the master will transmit a STOP condition and will not initiate a new START
        // condition until this flag has been cleared.
    	LPI2C0->MCR |= (1 << 8);  // reset transmit FIFO
    	LPI2C0->MCR |= (1 << 9);  // reset receive  FIFO
        LPI2C0->MSR = 0x400;      // clear NDF
    }
    if(LPI2C0->MSR & (1 << 11))
    {PTD->PCOR |= 1<<PTD15;
        // Master has lost arbitration
        // START or STOP condition detected and not generated by LPI2C master
        // Transmitting data on SDA and different value being received
        LPI2C0->MSR = 0x800;      // clear ALF
    }
    if(LPI2C0->MSR & (1 << 12))
    {PTD->PCOR |= 1<<PTD15;
        // Transmit FIFO requesting to transmit or receive data without a START condition
    	LPI2C0->MCR |= (1 << 8);  // reset transmit FIFO
    	LPI2C0->MCR |= (1 << 9);  // reset receive  FIFO
        LPI2C0->MSR = 0x1000;     // clear FEF
    }
    if(LPI2C0->MSR & (1 << 13))
    {
        // Pin low timeout has occurred
        // SCL (or SDA if MCFGR1[TIMECFG] is set) is low for
        // (MCFGR3[TIMELOW] * 256) prescaler cycles without a pin transition.

        // PTLF flag cannot be cleared while low condition continues
        // Restart master
    	LPI2C0->MCR &= ~(1 << 0);  // Disable master
    	LPI2C0->MSR = 0x2000;      // Clear LPTF
    	LPI2C0->MCR |=  (1 << 0);  // Enable master
    }
}


void LPI2C0_Slave_IRQHandler(void){
	if(LPI2C0->SSR & LPI2C_SSR_AVF_MASK){/*address valid interrupt*/
		//Verde();
		uint32_t address = (LPI2C0->SASR);
		if(address > 16000) GPIO_setPin(LED_RED);
		//Verde();
	}
	if(LPI2C0->SSR & LPI2C_SSR_RDF_MASK){/*Read interrupt*/
		//updated_data->slave_data = (uint8_t)(LPI2C0->SRDR & LPI2C_SRDR_DATA_MASK);
		//if(updated_data->slave_data == 170) Blue();
		if(slave_byte_count < SLAVE_FLOATS_RECEIVED*BYTESperFLOAT){//checar que no pase de 2
			data[slave_byte_count] = (uint8_t)(LPI2C0->SRDR & LPI2C_SRDR_DATA_MASK);
			if(data[slave_byte_count++] == 170) GPIO_setPin(LED_BLUE);
			delay_PDB(60000);

		}
		else{uint8_t clear = (uint8_t)(LPI2C0->SRDR & LPI2C_SRDR_DATA_MASK);}//leer el registro resetea el registro
		GPIO_setPin(LED_GREEN);
	}
	if(LPI2C0->SSR & LPI2C_SSR_BEF_MASK){
		LPI2C0->SSR |= LPI2C_SSR_BEF_MASK;//clear
		GPIO_setPin(LED_RED);//BIT error flag
		LPI2C0->SCR |= (1 << 9);
	}
	if(LPI2C0->SSR & LPI2C_SSR_FEF_MASK){
		LPI2C0->SSR |= LPI2C_SSR_FEF_MASK;//clear flag
		GPIO_setPin(LED_RED);//FIFO error flag
		//Blue();
		LPI2C0->SCR |= (1 << 9);
	}
	if(LPI2C0->SSR & LPI2C_SSR_AM0F_MASK){

		GPIO_setPin(LED_GREEN);//address 0 match
	}
	if(LPI2C0->SSR & LPI2C_SSR_SDF_MASK){//Stop detected
		LPI2C0->SSR |= LPI2C_SSR_SDF_MASK;//clear
		//Verde();
		//Rojo();
		slave_byte_count = 0;
	}
	if(LPI2C0->SSR & LPI2C_SSR_RSF_MASK){//Repeated start
		LPI2C0->SSR |= LPI2C_SSR_RSF_MASK;//clear
		//Verde();
		slave_byte_count = 0;
	}
//	Rojo();
}

uint8_t* get_data0(){
	//uint32_t join = data[1];
	//join |= data[0] << 8;
	return data;
}
uint8_t float_signals_update(float* steer,float* brake,float* acc){
	if(~(LPI2C0->SSR) & LPI2C_SSR_SBF_MASK){//si el esclavo está ocupado
		float u_signals[3];
		union{
			float f;
			unsigned long ul;
			unsigned char byte;
		}u;
		int i;
		for(i=0;i<SLAVE_FLOATS_RECEIVED;i++){
#if CHANGE_ENDIANNES
			u.ul = (data[3+i] << 24) | (data[2+i] << 16) | (data[1+i] << 8) | data[0+i];
#else
			u.ul = (data[0+i] << 24) | (data[1+i] << 16) | (data[2+i] << 8) | data[3+i];
#endif
			u_signals[i] = u.f;
		}
		*steer = u_signals[0];
		*brake = u_signals[1];
		*acc   = u_signals[2];
		return True;
	}
	return False;
}
