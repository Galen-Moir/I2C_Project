#include <MKL25Z4.H>
#include "I2C_LCD.h"
#include "i2c.h"
#include "delay.h"
#include <math.h>
#define VREF (3.2);
float voltage;
float result;
float scale = 3.2;
char read;
uint8_t lower;
// R values of potentiometer voltage divider
// R1 (TOP) = 1 008 000
// R2 (BOTTOM) = 1 008 000

void Init_ADC(void) {
	
	SIM->SCGC6 |= (1UL << SIM_SCGC6_ADC0_SHIFT); 
	ADC0->CFG1 = 0x9C; // low power config, long sample, single ended 16 bit 
	ADC0->SC2 = 0; // 
	
}

float ADC_Measure(void){
	unsigned result=0;
	ADC0-> SC1[0] = 0x00; //  single ended, no interrupts, input select for dadp0
		while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)){};
		result = ADC0->R[0];
		voltage = 3.2*result/0xffff;
		return voltage;
}

