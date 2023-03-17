
#include <MKL25Z4.H>
#include <stdio.h>
#include "gpio_defs.h"
#include "UART.h"
#include "NMEA.h"
#include "globals.h"
#include "I2C_LCD.h"
#include "LEDs.h"
#include "switches.h"
#include "timers.h"		
#include "delay.h"
#include "i2c.h"
#include "IMU.h"
#include "QMC5883L.h"
#include <stdint.h>
#include <stdio.h>
/*
185nF device capacitance
QMC
MinV = 1.65, 2.6mA
29k
IMU
MinV=2.375 3.8mA
IO
MIN= 2.31V, -1mA
R = (Vcc - Voh) / Ioh*Cbus
bus = 185nF
*/

int main (void) {
	uint8_t buff[40];
	uint8_t whoami;
	int j;
	int count =0;
	int Initalized =0;
	float Xdata[1000], Ydata[1000], Bdata[1000];
	i2c_init();
	init_I2C_LCD(LCD_ADDR_W);
	Clear_LCD(LCD_ADDR_W);
	//Init_IMU();
	Init_QMC();
	
	
	Init_UART2(9600);
	__enable_irq();
	

	while (1) {
		//Clear_LCD(LCD_ADDR_W);
		Set_Cursor(LCD_ADDR_W,0,0);
		//PrintIMUSensors(); // after calibration, prints the accelerometer and gyro sensor readings from IMU
		//readQMCSensors(); // prints un calibrated QMC mag sensor values
		//readCalQMCSensors(); // prints calibrated QMC mag sensor values
		TASK_Process_NMEA_Sentence();
		
		if( (cur_lon || cur_lat) != 0){
		Compass_Bearing();
		magnetic_bearingGPS(cur_lat,cur_lon,46.180172560280916, -75.74213586556242);
		compare_Bearing();
		Delay(200);
		}	
		
		//for(j=0;j<1000;j++){
		//Set_Cursor(LCD_ADDR_W,0,0);
		//Compass_Bearing(j);
		//Delay(50);
		//}
		
		//sprintf(buff, "%f ",magnetic_bearingGPS(40.7128, -74.0060, 34.0522, -118.2437));
		//Print_I2C_LCD_String(LCD_ADDR_W,buff);
		
		/*
		if (new_GPS_data) {

			if (GPS_fix_quality == Q_NONE) {
				Clear_LCD(LCD_ADDR_W);
				Set_Cursor(LCD_ADDR_W,0,0);
				Print_I2C_LCD_String(LCD_ADDR_W," No Fix ");
				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "%2d Sats ", GPS_num_sats);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
			} else {
				Clear_LCD(LCD_ADDR_W);
				Set_Cursor(LCD_ADDR_W,0,0);
				sprintf(buff, "%f", cur_lat);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);

				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "%f", cur_lon);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
				
				
				
				
			}
			
		}*/
		
	}
}


