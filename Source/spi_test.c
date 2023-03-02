/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include "gpio_defs.h"
#include "UART.h"
#include "LCD_4bit.h"
#include "LEDs.h"
#include "switches.h"
#include "timers.h"		
#include "delay.h"

#define CS_POS 0
#define END_CHAR 0x06

void spi_init_master( void){
	
	SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK; // Turn on clock to SPI0 module

  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; // Turn on clock to Port D module

	SPI0->C1 = 0x54; //set SPI0 control register 1 for master mode 
	SPI0->C2 = 0x80; //set SPI0 control register 2 for master mode 
	SPI0->BR = 0x24; //use baud rate of 250kHz (24MHz / (2+1) / 2^(4+1) = 250kHz)
	
	SPI0->M = END_CHAR; //set match register to ack character to recognize the end of messages
	
	PORTD->PCR[1] = PORT_PCR_MUX(2); // PTD1 pin is SPI0 CLK line
	PORTD->PCR[2] = PORT_PCR_MUX(2); 
	PORTD->PCR[3] = PORT_PCR_MUX(2);  
	
	//chip select is configured as GPIO from master's side, so that it can be manually controlled
	PORTD->PCR[CS_POS] = PORT_PCR_MUX(1);  // slow slew rate is enabled by default
	
	PTD->PDDR |= MASK(CS_POS);
	PTD->PSOR |= MASK(CS_POS); //set CS to 1
	
}

void spi_init_slave( void){
	
	SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK; // Turn on clock to SPI1 module
	
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; // Turn on clock to Port D module
	
	PORTD->PCR[4] = PORT_PCR_MUX(2);
	PORTD->PCR[5] = PORT_PCR_MUX(2); 
	PORTD->PCR[6] = PORT_PCR_MUX(2); 
	PORTD->PCR[7] = PORT_PCR_MUX(2); 

	SPI1->C1 = 0xc4; //set SPI1 control register 1 for slave mode (with SPRF interrupt enabled)
	SPI1->C2 = 0x80; //set SPI1 control register 2 for slave mode 
	SPI1->BR = 0x24; //use baud rate of 250kHz (24MHz / (2+1) / 2^(4+1) = 250kHz)
	SPI1->M = END_CHAR; //set match register to ack character to recognize the end of messages
	
	while(!(SPI1->S & SPI_S_SPTEF_MASK)); // while not all of data is sent from buffer
	// SPTEF must be read before sending data, otherwise the write is ignored
	SPI1->D = 0;//initialize slave data buffer to 0
	
	/* Enable Interrupts */
	NVIC_SetPriority(SPI1_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(SPI1_IRQn); 
	NVIC_EnableIRQ(SPI1_IRQn);
 
}

void SPI1_IRQHandler(void){ //ISR for slave that recieves a value from master, increments it and writes the new value into its own data buffer
	char output;
	NVIC_ClearPendingIRQ(SPI1_IRQn);
	while(!(SPI1->S & SPI_S_SPRF_MASK));
	output	= SPI1->D;
	while(!(SPI1->S & SPI_S_SPTEF_MASK));
	SPI1->D = output + 1; //copies the value just read  plus 1 back into data buffer
}

void spi_transfer_byte(char input, char* output){//tranfers a byte to the slave and receives a byte from the slave
	PTD->PCOR |= MASK(CS_POS); //set CS to 0
	while(!(SPI0->S & SPI_S_SPTEF_MASK)); // while not all of data is sent from buffer
	// SPTEF must be read before sending data, otherwise the write is ignored
	SPI0->D = input;
	while(!(SPI0->S & SPI_S_SPRF_MASK)); //wait until read buffer is full
	*output = SPI0->D; // reading data resets SPRF 
	PTD->PSOR |= MASK(CS_POS); //set CS to 1
}


int main(){
	
	char x, y, z;
	
	spi_init_master();
	spi_init_slave();
	
	spi_transfer_byte(1, &y);
	spi_transfer_byte(9, &y);
	spi_transfer_byte(3, &y);
	spi_transfer_byte(5, &y);
	
}

