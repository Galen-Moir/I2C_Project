#include <MKL25Z4.H>
#include "Delay.h"

void Delay (float dly) {
	// core clock is 48MHz by default, for loop takes 6 cycles to complete. Therefore Delay(1) takes 1.25 milliseconds
	// if dly is set to 10000. Setting to 8000 so when passed 1, 1ms is used 
  volatile uint32_t t;

	for (t=dly*8000; t>0; t--)
		;
}
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
