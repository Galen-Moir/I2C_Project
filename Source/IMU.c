#include "timers.h"
#include "MKL25Z4.h"
#include <math.h>
#include "I2C_LCD.h"
#include "IMU.h"
#include "i2c.h"
#include "delay.h"
#include <stdio.h>
/*
hold time is 300 ns and min start time is 5us
so define hold time as 5.3us or 0.0053ms
round to 0.0055
datashet:https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
*/
typedef int16_t fixed_point_t;
#define FIXED_POINT_FRACTIONAL_BITS 2
#define FIXED_POINT_ACCEL_SCALING_FACTOR (1 << FIXED_POINT_FRACTIONAL_BITS) / 16384.0
#define FIXED_POINT_GYRO_SCALING_FACTOR (1 << FIXED_POINT_FRACTIONAL_BITS) / 131

int16_t AccelOffset[3] ={0};
int16_t GyroOffset[3] = {0};
fixed_point_t Accel[3] ={0};
fixed_point_t Gyro[3] = {0};
// Define characteristics from hardcoded initalization if needed to be referenced. Must change if using different setup
float IMUTimeDelay = (0.0055);
float SamplingRate = (0.019); // 19ms, set in SMPRT_DIV register during init to compensate for filter set in CONFIG register
float GyroRange = (500); // +- 250 degrees / sec resolution, set in GYRO_CONFIG during init
float AccelRange = (4); // +- 2g register output 


void Init_IMU(void){
	WRITE_IMU(PWR_MGMT_1,0x80); // reset registers, just incase
	WRITE_IMU(SMPRT_DIV,0x12); // divide sampling rate by 19, once filter is on it divides 
	//gyro 1kz by 19 to not sample more then 18.6ms delay bottlenecks
	WRITE_IMU(CONFIG,0x05); // Set gyroscope digital low pass filter to 10hz bandwidth, causing 18.6ms delay
	WRITE_IMU(GYRO_CONFIG,0x00); // default range of 0 is +/- 250 degrees/sec, 180 is enough for rotation 
	//in spot on earth, 131 LSB sensitivity
	WRITE_IMU(ACCEL_CONFIG,0x00); // same as gyro, default is sufficient, +\- 2g
	WRITE_IMU(PWR_MGMT_1,0x00); // Write sleep bit low to wake up sensor, 8MHz oscillator
	calibrateIMU(AccelOffset,GyroOffset);
}



void WRITE_IMU(uint8_t address,uint8_t data){
	
	Delay(IMUTimeDelay);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = IMU_ADDR_W;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C1->D = data;
	I2C_WAIT;
	
	I2C_M_STOP;							/*send stop	*/
}

uint8_t READ_IMU(uint8_t address){
	uint8_t data;
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = IMU_ADDR_W;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C_M_RSTART;
	I2C1->D = IMU_ADDR_R;
	I2C_WAIT;
	
	I2C_REC;						   /*set to receive mode */
	NACK;									 /*set NACK after read	*/
	
	data = I2C1->D;					/*dummy read	*/
	I2C_WAIT;								/*wait for completion */
	
	I2C_M_STOP;							/*send stop	*/
	data = I2C1->D;
	/*read data	*/
	return data;
}

int16_t readIMUSensor(uint8_t regH, uint8_t regL)
{
    uint8_t data[2];
    data[0] = READ_IMU(regH);
		data[1] = READ_IMU(regL);
    return (data[0] << 8) | data[1];
}

void PrintIMUSensors(void)
{
	char buff[40];
	readIMUSensors(Accel,Gyro);    
				Set_Cursor(LCD_ADDR_W,0,0);
				sprintf(buff, "A: %0.2f %0.2f %0.2f", (float)Accel[0], (float)Accel[1], (float)Accel[2]);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "G: %0.2f %0.2f %0.2f", (float)Gyro[0], (float)Gyro[1], (float)Gyro[2]);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
	
	//sprintf(buff, "A: %d %d %d G: %d %d %d", Accel[0], Accel[1], Accel[2],Gyro[0], Gyro[1], Gyro[2]);
	//printf("%.*s\n",(int)sizeof(buff),buff);
}


void readIMUSensors(int16_t* Accel,int16_t* Gyro)
{
        Accel[0] = (fixed_point_t)( (readIMUSensor(ACCEL_XOUT_H, ACCEL_XOUT_L) + AccelOffset[0])*FIXED_POINT_ACCEL_SCALING_FACTOR);
        Accel[1] = (fixed_point_t)( (readIMUSensor(ACCEL_YOUT_H, ACCEL_YOUT_L) + AccelOffset[1])*FIXED_POINT_ACCEL_SCALING_FACTOR);
        Accel[2] = (fixed_point_t)( (readIMUSensor(ACCEL_ZOUT_H, ACCEL_ZOUT_L) + AccelOffset[2])*FIXED_POINT_ACCEL_SCALING_FACTOR);
        Gyro[0] = (fixed_point_t)( (readIMUSensor(GYRO_XOUT_H, GYRO_XOUT_L) + GyroOffset[0])*FIXED_POINT_GYRO_SCALING_FACTOR);
        Gyro[1] = (fixed_point_t)( (readIMUSensor(GYRO_YOUT_H, GYRO_YOUT_L)+ GyroOffset[1])*FIXED_POINT_GYRO_SCALING_FACTOR);
        Gyro[2] = (fixed_point_t)( (readIMUSensor(GYRO_ZOUT_H, GYRO_ZOUT_L)+ GyroOffset[2])*FIXED_POINT_GYRO_SCALING_FACTOR);
        
}

void calibrateIMU(int16_t* AccelOffset,int16_t* GyroOffset) {
  int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
  float accel_x_offset = 0;
  float accel_y_offset = 0;
  float accel_z_offset = 0;
	float gyro_x_offset = 0;
  float gyro_y_offset = 0;
  float gyro_z_offset = 0;
	int i;
	for (i=0 ; i < 1000; i++) {
    accel_x = readIMUSensor(ACCEL_XOUT_H, ACCEL_XOUT_L);
    accel_y = readIMUSensor(ACCEL_YOUT_H, ACCEL_YOUT_L);
    accel_z = readIMUSensor(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
		gyro_x = readIMUSensor(GYRO_XOUT_H, GYRO_XOUT_L);
    gyro_y = readIMUSensor(GYRO_YOUT_H, GYRO_YOUT_L);
    gyro_z = readIMUSensor(GYRO_ZOUT_H, GYRO_ZOUT_L);
		
    accel_x_offset += accel_x;
    accel_y_offset += accel_y;
    accel_z_offset += accel_z;
		gyro_x_offset += gyro_x;
    gyro_y_offset += gyro_y;
    gyro_z_offset += gyro_z;
 // prob un necessary, but delay entire route the time it takes for a new sample from gyro scope to occur due to resolution of measurement. In actuality, itd be 18.6ms - the time it takes to do the above commands
  }
  accel_x_offset /= 1000;
  accel_y_offset /= 1000;
  accel_z_offset /= 1000;
	gyro_x_offset /= 1000;
  gyro_y_offset /= 1000;
  gyro_z_offset /= 1000;
  
  // Set MPU6050 accelerometer offsets
	
  AccelOffset[0] = -accel_x_offset;
  AccelOffset[1] = -accel_y_offset;
  AccelOffset[2] = -accel_z_offset;
	GyroOffset[0] = -gyro_x_offset;
	GyroOffset[1] = -gyro_y_offset;
	GyroOffset[2] = -gyro_z_offset;
}

/*
void RollPitchYaw(void){
	// y axis is the forward direction of the chip, with its identifing dot in top left, moving in same plane, towards the side forward of the dot
	float roll, pitch;
	roll = atan2(accel_y, accel_z)
	pitch = atan2(-accel_x, sqrt(accel_y^2 + accel_z^2))
	
}
*/
	
