/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
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

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	uint8_t c='a', line=0, col=0;
	uint32_t n;
	uint8_t buff[40];
	int count =0;
	int Initalized =0;
	i2c_init();
	init_I2C_LCD(LCD_ADDR_W);
	Clear_LCD(LCD_ADDR_W);
	Init_IMU();
	
	
	Init_UART2(9600);
	__enable_irq();
	

	while (1) {
		TASK_Process_NMEA_Sentence();
		Clear_LCD(LCD_ADDR_W);
		Set_Cursor(LCD_ADDR_W,0,0);
		readSensors();
		// I2C1->F = (I2C_F_ICR(0x5) | I2C_F_MULT(0x1)); // 24m / (30*2) = 400K
		/*
		Print_I2C_LCD_String(LCD_ADDR_W,"G:");
		sprintf(buff, "%x", READ_IMU( GYRO_XOUT_H ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		sprintf(buff, "%x", READ_IMU( GYRO_XOUT_L ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		
		Print_I2C_LCD_String(LCD_ADDR_W," ");
		sprintf(buff, "%x", READ_IMU( GYRO_YOUT_H ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		sprintf(buff, "%x", READ_IMU( GYRO_YOUT_L ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		
		Print_I2C_LCD_String(LCD_ADDR_W," ");
		sprintf(buff, "%x", READ_IMU( GYRO_ZOUT_H ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		sprintf(buff, "%x", READ_IMU( GYRO_ZOUT_L ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		
		Set_Cursor(LCD_ADDR_W,0,1);
		Print_I2C_LCD_String(LCD_ADDR_W,"A:");
		sprintf(buff, "%x", READ_IMU( ACCEL_XOUT_H ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		sprintf(buff, "%x", READ_IMU( ACCEL_XOUT_L ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		
		Print_I2C_LCD_String(LCD_ADDR_W," ");
		sprintf(buff, "%x", READ_IMU( ACCEL_YOUT_H ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		sprintf(buff, "%x", READ_IMU( ACCEL_YOUT_L ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		
		Print_I2C_LCD_String(LCD_ADDR_W," ");
		sprintf(buff, "%x", READ_IMU( ACCEL_ZOUT_H ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		sprintf(buff, "%x", READ_IMU( ACCEL_ZOUT_L ) );
		Print_I2C_LCD_String(LCD_ADDR_W,buff);
		*/
		Delay(200);
		/*
		if (new_GPS_data) {
			new_GPS_data = 0;
			
			if (GPS_fix_quality == Q_NONE) {
				Set_Cursor(LCD_ADDR_W,0,0);
				Print_I2C_LCD_String(LCD_ADDR_W," No Fix ");
				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "%2d Sats ", GPS_num_sats);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
			} else {
				
				Set_Cursor(LCD_ADDR_W,0,0);
				sprintf(buff, "%f", cur_lat);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);

				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "%f", cur_lon);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
				
				
				
					//if(Initalized == 0) {
					//Initalized = 1;
					//	Init_RTC_SEC_INT();
					//}
				
				
				
			}
		}
		*/
	}
}


// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
