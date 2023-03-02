#include <MKL25Z4.H>
#include "i2c.h"

#define InterruptEnable = 1;
//init i2c1
void i2c_init(void)
{
	//clock i2c peripheral and port C
	SIM->SCGC4 |= (SIM_SCGC4_I2C1_MASK);
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);
	
	
	//set pins to I2C function
	PORTC->PCR[10] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK);
	PORTC->PCR[10] |= PORT_PCR_MUX(2);
	PORTC->PCR[11] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK);
	PORTC->PCR[11] |= PORT_PCR_MUX(2);
		

	
	//set to 100k baud
	//baud = bus freq/(scl_div*mul)
 	//100k = 24M/(1*240); icr=0x1F sets scl_div to 240
 	I2C1->F = (I2C_F_ICR(0x1F) | I2C_F_MULT(0));
	
	//enable i2c and set to master mode
	I2C1->C1 |= (I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK);
}


//send start sequence
void i2c_start()
{
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
}

//send device and register addresses
void i2c_read_setup(uint8_t dev, uint8_t address)
{
	I2C1->D = dev;			  /*send dev address	*/
	I2C_WAIT;							/*wait for completion */
	
	I2C1->D =  address;		/*send read address	*/
	I2C_WAIT;							/*wait for completion */
		
	I2C_M_RSTART;				   /*repeated start */
	I2C1->D = (dev|0x1);	 /*send dev address (read)	*/
	I2C_WAIT;							 /*wait for completion */
	
	I2C_REC;						   /*set to receive mode */

}

//read a byte and ack/nack as appropriate
uint8_t i2c_repeated_read(uint8_t isLastRead)
{
	uint8_t data;
	
	if(isLastRead)	{
		NACK;								/*set NACK after read	*/
	} else	{
		ACK;								/*ACK after read	*/
	}
	
	data = I2C1->D;				/*dummy read	*/
	I2C_WAIT;							/*wait for completion */
	
	if(isLastRead)	{
		I2C_M_STOP;					/*send stop	*/
	}
	data = I2C1->D;				/*read data	*/

	return  data;					
}



//////////funcs for reading and writing a single byte
//using 7bit addressing reads a byte from dev:address
uint8_t i2c_read_byte_intReg(uint8_t dev, uint8_t address)
{
	uint8_t data;
	
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
	I2C1->D = dev;			  /*1. send dev address	*/
	I2C_WAIT;							/*wait for completion */
	
	I2C1->D =  address;		/*2. send read address	*/
	I2C_WAIT;							/*wait for completion */
		
	I2C_M_RSTART;				   /*repeated start */
	I2C1->D = (dev|0x1);	 /*3. send dev address (read)	*/
	I2C_WAIT;							 /*wait for completion */
	
	I2C_REC;						   /*set to receive mode */
	NACK;									 /*set NACK after read	*/
	
	data = I2C1->D;					/*dummy read	*/
	I2C_WAIT;								/*wait for completion */
	
	I2C_M_STOP;							/*send stop	*/
	data = I2C1->D;					/*read data	*/

	return data;
}

uint8_t i2c_read_byte(uint8_t addr)
{
	uint8_t data;
	
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
	
	I2C1->D =  addr;		/*send device address	*/
	I2C_WAIT;							/*wait for completion */
	
	I2C_REC;						   /*set to receive mode */
	NACK;									 /*set NACK after read	*/
	
	data = I2C1->D;					/*dummy read	*/
	I2C_WAIT;								/*wait for completion */
	
	I2C_M_STOP;							/*send stop	*/
	data = I2C1->D;					/*read data	*/

	return data;
}

//using 7bit addressing writes a byte data to dev:address
void i2c_write_byte_intReg(uint8_t dev, uint8_t address, uint8_t data)
{
	
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
	I2C1->D = dev;			  /*send dev address	*/
	I2C_WAIT;						  /*wait for ack */
	
	I2C1->D =  address;		/*send write address	*/
	I2C_WAIT;
		
	I2C1->D = data;				/*send data	*/
	I2C_WAIT;
	I2C_M_STOP;
	
}

//Using specific read or write addresses to communicate to device with no internal register selection
void i2c_write_byte(uint8_t addr, uint8_t data)
{
	
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
	I2C1->D = addr;			  /*send dev address	*/
	I2C_WAIT;						  /*wait for ack */
	I2C1->D =  data;		/*send write address	*/
	I2C_WAIT;
	I2C_M_STOP;
	
}



