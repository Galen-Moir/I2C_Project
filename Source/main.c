/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <math.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "IO_expander.h"
#include "I2C_LCD.h"
#include "PIT.h"
#include "mma8451.h"
#include "delay.h"

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

	i2c_prestart_LCD();
	i2c_init();																/* init i2c	*/
	INT_IO_init (); // init IO expander
	init_I2C_LCD(LCD_ADDR_W);
	PIT_init();

	while (1) {
		Delay(1);
	}
	
}
