#include "timers.h"
#include "MKL25Z4.h"
#include <math.h>
#include "I2C_LCD.h"
#include "IMU.h"
#include "i2c.h"
#include "delay.h"
#include "Magnetometer.h"


/*
Requires a hold time of 1.2us and start time of 0.6us equal to 1.8us, rounding to 2us
2us/1ms=0.002
https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
*/
int16_t Mag[3] ={0};
int8_t Who[4] ={0};
void Init_MAG(void){
	// https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
		//WRITE_MAG(HMC5883L_CONFIG_A,0xE5); // 0 11(8 samples averaged) 100(Default 15hz sampling, 101 is 35 and 110 is 75 if need be) 00 ( No bias to readings added YET), 11100101 = 0xE5
		//WRITE_MAG(HMC5883L_CONFIG_B,0x40); // 010 (gain of 1.9, with earths magnetic field being weakin comparison to motors, higher gain may be useful but then again, amplifys nosie, 0 0000, other bits must be 0. 0100 0000=0x40
		//WRITE_MAG(HMC5883L_MODE,0x00); // MSB can toggle "high speed" i2c at 3400KHz, bits 0-1 dictate mode. 00 is continuous reading, 01, is single reading mode. Continuous is fine and non highspeed is fine, 0x00
	
	//Example initalization
	//WRITE_MAG(HMC5883L_CONFIG_A,0x70);
	//Delay(10);
	//WRITE_MAG(HMC5883L_CONFIG_B,0xA0); 
	//Delay(10);
	Delay(1);
	WRITE_MAG(HMC5883L_MODE,0x00);
	Delay(10);
}

int16_t readMAGSensor(uint8_t regH, uint8_t regL)
{
	
    uint8_t data[2];
    data[0] = READ_MAG(regH);
		data[1] = READ_MAG(regL);
    return (data[0] << 8) | data[1];
}

void WhoAmIread()
{

	int i;
	uint8_t data[4];
	
	i2c_start();
	i2c_read_setup(HMC5883L_Address, HMC5883L_STATUS);
	
	// Read 3 bytes in repeated mode
	for( i=0; i<3; i++)	{
		data[i] = i2c_repeated_read(0);
	}
	// Read last byte ending repeated mode
	data[i] = i2c_repeated_read(1);
	

	Who[0]= data[0];
	Who[1] = data[1];
	Who[2] = data[2];	
	Who[3] = data[3];	
	}

void readMAGSensors(int16_t* Mag)
{
				
	int i;
	uint8_t data[6];
	
	Delay(MAGTimeDelay); // may need to make bigger
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
	
	I2C1->D = HMC5883L_R;		// read command
	I2C_WAIT;						 /*wait for completion */	
	I2C_REC;						   /*set to receive mode */
	
	// Read five bytes in repeated mode
	for( i=0; i<5; i++)	{
		if(i==5)	{
		NACK;								/*set NACK after read	*/
	} else	{
		data[i] = I2C1->D;
		ACK;								/*ACK after read	*/
		I2C_WAIT;	
	}
	
	if(i == 5)	{
		I2C_M_STOP;					/*send stop	*/
	}
	data[i] = I2C1->D;				/*read data	*/				
	}
	
	/*
	Delay(MAGTimeDelay); // may need to make bigger
	I2C_TRAN;							//set to transmit mode 
	I2C_M_START;
	I2C1->D = HMC5883L_W;		// write command
	I2C_WAIT;							
	I2C1->D =  (0x03);		//reset pointer
	I2C_WAIT;	
	I2C_M_STOP;	
	*/
	
	Mag[0] = ( (data[0]<<8) | data[1]);
	Mag[1] = ( (data[2]<<8) | data[3]);
	Mag[2] = ( (data[4]<<8) | data[5]);		
}

void PrintMAGSensors(void)
{
	uint8_t buff[40];
	readMAGSensors(Mag);    
				Set_Cursor(LCD_ADDR_W,0,0);
				sprintf(buff, "X:%d Z:%d" ,Mag[0], Mag[1]);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "Y:%d", Mag[2]);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
}


uint8_t READ_MAG(uint8_t address){
	
	uint8_t data;
	Delay(MAGTimeDelay);
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = HMC5883L_R;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C_M_RSTART;
	
	I2C_REC;						   /*set to receive mode */
	NACK;									 /*set NACK after read	*/
	
	data = I2C1->D;					/*dummy read	*/
	I2C_WAIT;								/*wait for completion */
	
	I2C_M_STOP;							/*send stop	*/
	data = I2C1->D;
	return data;
}

void WRITE_MAG(uint8_t address,uint8_t data){
	
	Delay(MAGTimeDelay);
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = HMC5883L_Address;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C1->D = data;
	I2C_WAIT;
	
	I2C_M_STOP;							/*send stop	*/
}


