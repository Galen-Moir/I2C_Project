#include "timers.h"
#include "MKL25Z4.h"
#include <math.h>
#include "I2C_LCD.h"
#include "IMU.h"
#include "i2c.h"
#include "delay.h"

void Init_IMU(void){
	WRITE_IMU(PWR_MGMT_1,0x80); // reset registers, just incase
	WRITE_IMU(SMPRT_DIV,0x12); // divide sampling rate by 19, once filter is on it divides gyro 1kz by 19 to not sample more then 18.6ms delay bottlenecks
	WRITE_IMU(CONFIG,0x05); // Set gyroscope digital low pass filter to 10hz bandwidth, causing 18.6ms delay
	WRITE_IMU(GYRO_CONFIG,0x00); // default range of 0 is +/- 250 degrees/sec, 180 is enough for rotation in spot on earth, 131 LSB sensitivity
	WRITE_IMU(ACCEL_CONFIG,0x00); // same as gyro, default is sufficient, +\- 2g
	WRITE_IMU(PWR_MGMT_1,0x00); // write register low since in sleep mode by default, 16384 LSB sensitvity 
	
}



void WRITE_IMU(uint8_t address,uint8_t data){
	
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = IMU_ADDR_W;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C1->D = data;
	I2C_WAIT;
	
	I2C_M_STOP;							/*send stop	*/
	Delay(5);
}

uint8_t READ_IMU(uint8_t address){
	uint8_t data;
	
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
	data = I2C1->D;					/*read data	*/
	Delay(5);
	return data;
}

int16_t readSensor(uint8_t regH, uint8_t regL)
{
    uint8_t data[2];
    data[0] = READ_IMU(regH);
		data[1] = READ_IMU(regL);
    return (data[0] << 8) | data[1];
}

void readSensors(void)
{
	uint8_t buff[40];
        int16_t accelX = readSensor(ACCEL_XOUT_H, ACCEL_XOUT_L);
        int16_t accelY = readSensor(ACCEL_YOUT_H, ACCEL_YOUT_L);
        int16_t accelZ = readSensor(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
        int16_t gyroX = readSensor(GYRO_XOUT_H, GYRO_XOUT_L);
        int16_t gyroY = readSensor(GYRO_YOUT_H, GYRO_YOUT_L);
        int16_t gyroZ = readSensor(GYRO_ZOUT_H, GYRO_ZOUT_L);
        
				Set_Cursor(LCD_ADDR_W,0,0);
				sprintf(buff, "A: %d %d %d", accelX, accelY, accelZ);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
				Set_Cursor(LCD_ADDR_W,0,1);
				sprintf(buff, "G: %d %d %d", gyroX, gyroY, gyroZ);
				Print_I2C_LCD_String(LCD_ADDR_W,buff);
}

void calibrateIMU() {
  int16_t accel_x, accel_y, accel_z;
  float accel_x_offset = 0;
  float accel_y_offset = 0;
  float accel_z_offset = 0;
	
	for (int i = 0; i < 1000; i++) {
    accel_x = readSensor(ACCEL_XOUT_H, ACCEL_XOUT_L);
    accel_y = readSensor(ACCEL_YOUT_H, ACCEL_YOUT_L);
    accel_z = readSensor(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
    accel_x_offset += accel_x;
    accel_y_offset += accel_y;
    accel_z_offset += accel_z;
    delay(1);
  }
  accel_x_offset /= 1000;
  accel_y_offset /= 1000;
  accel_z_offset /= 1000;
  
  // Set MPU6050 accelerometer offsets
	
  MPU6050_setXAccelOffset(mpu, -accel_x_offset / 8);
  MPU6050_setYAccelOffset(mpu, -accel_y_offset / 8);
  MPU6050_setZAccelOffset(mpu, (16384 - accel_z_offset) / 8);
	Z is different because range is 0 to 2g, so offest hast to be positive gpt says
	
}
	
