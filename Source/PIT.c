#include <MKL25Z4.H>
#include "I2C_LCD.h"
#include "i2c.h"
#include "delay.h"
#include "IO_expander.h"
#include "ADC.h"
#include <math.h>

char tenth =0x30;
char sec = 0x30;
char tensec = 0x30;
char hundredsec = 0x30;

void PIT_init(void){
SIM->SCGC6 |= SIM_SCGC6_PIT_MASK; // enable pit clock
PIT->MCR &= ~PIT_MCR_MDIS_MASK;// enable module (active low setting)
PIT->MCR |= PIT_MCR_FRZ_MASK; // freeze timer during debug
PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(0x249EFF); // set load value for 0.1s accuracy
PIT->CHANNEL[0].TCTRL &= PIT_TCTRL_CHN_MASK; // set chain off
	
//let pit generate intterupt req
PIT-> CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;

NVIC_SetPriority(PIT_IRQn, 0); //0, 64 , 128 or 192
NVIC_ClearPendingIRQ(PIT_IRQn);// clear pending flag
NVIC_EnableIRQ(PIT_IRQn); // enable pit in nvic

}

void Start_PIT(void) {
// Enable counter
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void Stop_PIT(void) {
// stop counter
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}

void PIT_IRQHandler(void){

// gonna have to work on C variable data structure formatting
	NVIC_ClearPendingIRQ(PIT_IRQn);// clear int flag

		if(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK){
				PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK; // clear channel int flag
		

				tenth = tenth+0x01;
				if(tenth == 0x3A){ // tenth == to char(10) maybe> check that email
				tenth = 0x30;
				sec = sec + 0x01;
				}
				if(sec == 0x3A){
				sec = 0x30;
				tensec = tensec + 0x01;
				}
				if(tensec == 0x3A){
				tensec = 0x30;
				hundredsec = hundredsec + 0x01;
				}
				// Should set cursor to third from right bottom row and write sec + "." + tenth normally,
				// set cursor to fourth from right bottom
				// then print tensec but have the the screen scroll to left, will have to change setting and reset after
					}
	}
