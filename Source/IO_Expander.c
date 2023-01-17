#include <MKL25Z4.H>
#include "mma8451.h"
#include "i2c.h"
#include "I2C_LCD.h"
#include "IO_Expander.h"
#include "PIT.h"
#include "delay.h"
#include <math.h>
#include <stdio.h> // for sprintf

char str[5];


//IO data
volatile uint8_t IOdata;




// uint8_t count (uint8_t x);

// uint8_t IO_READ_1Byte(uint8_t addr,volatile uint8_t x);

void INT_IO_init (void){
SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; // enable INT pin port clock

/* Select GPIO and enable pull-up resistors and interrupts 
		on falling edges for pins connected to switches */
	PORTA->PCR[INT_PIN_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x08); // Trying a logic zero interrupt for one interrupt per change for now

	/* Set port A INT bit to inputs */
	PTA->PDDR &= ~MASK(INT_PIN_POS);
	

NVIC_SetPriority(PORTA_IRQn, 0); //0, 64 , 128 or 192
NVIC_ClearPendingIRQ(PORTA_IRQn);// clear pending flag
NVIC_EnableIRQ(PORTA_IRQn); // enable pit in nvic

}

void PORTA_IRQHandler(void){

	NVIC_ClearPendingIRQ(PORTA_IRQn); // Clear pending status
	IOdata = i2c_read_byte(IOEX_ADDR_R); // Read IO data and store it
	Set_Cursor(LCD_ADDR_W,0,0);
	
	sprintf(str, "%u%u.%u%u", hundredsec,tensec,sec,tenth); // store pit time as string
	Print_I2C_LCD_String(LCD_ADDR_W,str); // print PIT time
	hundredsec = 0; // reset pit timer variables
	tensec = 0;
	sec = 0;
	tenth =0;
	
	Set_Cursor(LCD_ADDR_W,0,1);
	Print_I2C_LCD_IO_READ(LCD_ADDR_W);
	Set_Cursor(LCD_ADDR_W, 9,1);
	if (IOdata == 0x7F) { // If light sensor low
		Print_I2C_LCD_String(LCD_ADDR_W, "Light");
	}
	if (IOdata == 0xBF) { // If button low
		Print_I2C_LCD_String(LCD_ADDR_W, "Button");
	}
	if (IOdata == 0x3F) { // If both low
		Print_I2C_LCD_String(LCD_ADDR_W, "Both");
	}
	// clear status flags 
	PORTA->ISFR = 0xffffffff;
}

