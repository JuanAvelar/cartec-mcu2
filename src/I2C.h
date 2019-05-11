/*
 * I2C.h
 *
 *  Created on: 19/02/2019
 *      Author: Juan Avelar
 */

#ifndef I2C_H_
#define I2C_H_
/*******************************************************************************
* Constants and macros
*******************************************************************************/
enum status{
	OK,
	BUSY,
	NO_DATA_RECEIVED
};



#define BUSY_TIMEOUT		5000
#define READING_TIMEOUT		10000

/*******************************************************************************
* Functions
*******************************************************************************/
uint8_t I2C_read_reg			(uint8_t address, uint8_t reg, uint8_t *p_buffer);
uint8_t I2C_read_multiple_reg	(uint8_t address, uint8_t reg, uint8_t *p_buffer, uint8_t n_bytes);
uint8_t I2C_read				(uint8_t *p_buffer, uint8_t n_bytes);
uint8_t I2C_write_reg			(uint8_t address, uint8_t reg, uint16_t val);
uint8_t I2C_write_reg_32		(uint8_t address, uint8_t reg, uint32_t val);

uint8_t I2C_bus_busy			(void);
void 	I2C_start_NACK			(uint8_t address);
void 	I2C_start_ACK			(uint8_t address);
void 	I2C_stop				(void);
void 	I2C_write_byte			(uint8_t data);
void 	LPI2C0_init_master		(void);
void 	LPI2C0_IRQs_init		(void);
void	LPI2C0_Slave_IRQHandler	(void);
void	LPI2C1_Slave_IRQHandler	(void);
void 	LPI2C0_Master_IRQHandler(void);
void Te_ordeno_que_te_inicies_esclavo0(int direction);
void Te_ordeno_que_te_inicies_esclavo1(int direction);
//tener en cuenta el tamaño del arreglo de donde se sacar el puntero para no accesar memoria prohibida.
uint8_t* get_data0	(void);
uint8_t float_signals_update(float* steer,float* brake,float* acc);

#endif /* I2C_H_ */
