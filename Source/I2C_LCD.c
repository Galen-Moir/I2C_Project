#include <MKL25Z4.H>
#include "I2C_LCD.h"
#include "i2c.h"
#include "delay.h"
#include <math.h>
#include "IO_expander.h"

// bit structure for LCD connections
/*  P0:RS
		P1:RW
		P2:E
		P3:BT
		P4:D4
		P5:D5
		P6:D6
		P7:D7
		http://www.handsontec.com/dataspecs/module/I2C_1602_LCD.pdf
		Shows pin connection, were using the PCF8574, not PCD 8574A though
		https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf
		
		RS	R/~W	Operation
		0	0	Write instruction
		0	1	Read busy flag and address counter 
		1	0	Write data
		1	1	Read data

		
*/ 

void clear_device (uint8_t cycles, uint8_t bit){
	volatile uint8_t t;
	
	for(t = 0; t <=cycles ; t++) {
	
	PTC-> PSOR = (PTC->PSOR ^ PTC->PSOR) ;
	Delay(10);
	PTC->PSOR |= (1 << (bit)); 
	Delay(10);
	PTC-> PSOR = (PTC->PSOR ^ PTC->PSOR);
		
	}
}
	

void LCD_Write_4bit_CMD(uint8_t addr, uint8_t c)
{
	// input must be in form of 0x0c or 0xc
	// For instructions RS & RW =0, Backlight is high
	uint8_t EHi = 0x0C;
	uint8_t ELo = 0x08;
	
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = addr;
	I2C_WAIT;
	I2C1->D = ((c<<4) | ELo);
	I2C_WAIT;
	I2C1->D = ((c<<4) | EHi);
	I2C_WAIT;
	I2C_M_STOP;
	Delay(5);
	
}

void LCD_Write_8bit_CMD(uint8_t addr,uint8_t c)
{
	uint8_t EHi = 0x0C;
	uint8_t ELo = 0x08;
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = addr;
	I2C_WAIT;
	I2C1->D = ((c & 0xF0) | ELo);
	I2C_WAIT;
	I2C1->D = ((c & 0xF0) | EHi);
	I2C_WAIT;
	I2C1->D = (((c<<4) & 0xF0) | ELo);
	I2C_WAIT;
	I2C1->D = (((c<<4) & 0xF0) | EHi);
	I2C_WAIT;
	I2C_M_STOP;
	Delay(5);
}
void LCD_Write_8bit(uint8_t addr, uint8_t c)
{
	uint8_t EHi = 0x0D;
	uint8_t ELo = 0x09;
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = addr;
	I2C_WAIT;
	I2C1->D = ((c & 0xF0) | EHi);
	I2C_WAIT;
	I2C1->D = ((c & 0xF0) | ELo);
	I2C_WAIT;
	I2C1->D = (((c<<4) & 0xF0) | EHi);
	I2C_WAIT;
	I2C1->D = (((c<<4) & 0xF0) | ELo);
	I2C_WAIT;
	I2C_M_STOP;
	Delay(5);
}

void lcd_putchar(uint8_t addr, char c)
{
 LCD_Write_8bit(addr,c);
}

// havent used this yet. May not need
void wait_read_BF()
{
	uint8_t data;
	
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = LCD_ADDR_R;
	I2C_WAIT;
	
	I2C_M_RSTART;
	I2C1->D = LCD_ADDR_R;
	I2C_WAIT;
	
	I2C_REC;
	NACK;
	
	data = I2C1->D;
	I2C_WAIT;
	
	I2C_M_STOP;
	data = I2C1->D;
	if ((data & 0x80) == 0x80){
		wait_read_BF();
	}
}

void Set_Cursor(uint8_t addr, uint8_t column, uint8_t row)
{
	int8_t address; 
	address = (row*0x40) + column;
	address |= 0x80;
	LCD_Write_4bit_CMD(addr, address);
	
}
void Clear_LCD(uint8_t addr)
{
	LCD_Write_8bit_CMD(addr, 0x01);
	Set_Cursor(addr,0,0);
}

void Print_I2C_LCD_String(uint8_t addr,char *string)
{
	while(*string) {
		lcd_putchar(addr, *string++);
	}
}

void Print_I2C_LCD_IO_READ(uint8_t addr){
	
	for(int i =0; i < 8; i++){ // iterate through 8 bit shared variable IOdata from IOexpander IRQ
		if((IOdata & MASK(i))== 1){ 
			LCD_Write_8bit(addr, 0x31); // print a ASCII 1
			}
		if((IOdata & MASK(i))== 0){
			LCD_Write_8bit(addr, 0x30); // print a ASCII 0
			}
	}
	
}

void init_I2C_LCD(uint8_t addr)
{
	// Delay(100);
	LCD_Write_4bit_CMD(addr,0x3); // set 8 bit
	Delay(100);
	LCD_Write_4bit_CMD(addr,0x3); // set 8 bit
	Delay(10);
	LCD_Write_4bit_CMD(addr,0x3); // set 8 bit
	LCD_Write_4bit_CMD(addr,0x2); // set to 4 bit
	LCD_Write_8bit_CMD(addr,0x28); // set 2 lines
	LCD_Write_8bit_CMD(addr,0x0C); // 
	LCD_Write_8bit_CMD(addr,0x06);
	LCD_Write_8bit_CMD(addr,0x80);
	
}

void i2c_prestart_LCD(void)
{
	//Clear possible unknown FSM state of previously attatched LCD on preattatched pin
	
	// make future CLK GPIO
	PORTC->PCR[10] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK ;
	// Set port pin to outputs
	PTC->PDDR |= MASK(SCL_POS); 
	// clumsy approach to make sure I2C peripheral isnt stuck waiting for command
	// https://www.i2c-bus.org/i2c-primer/analysing-obscure-problems/blocked-bus/
	clear_device(16,10);
	// revert config
	PORTC -> PCR[10] = 0;

	
}

