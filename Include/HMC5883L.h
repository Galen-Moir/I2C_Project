#ifndef HMC5883L_H
#define HMC5883L_H
#include "MKL25Z4.h"

// datasheet :https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf

#define HMC5883L_Address 		   (0x1E) // 7 bit b (0x1E) 001 1110 
#define HMC5883L_W 		 			   (0x3C) // 1110 1100 -> 0011 1100 (3C Write)
#define HMC5883L_R 		 			 	 (0x3D) // 1110 1101 -> 0011 1101 (3D read)

#define HMC5883L_CONFIG_A      (0x00)
#define HMC5883L_CONFIG_B      (0x01)
#define HMC5883L_MODE          (0x02)
#define HMC5883L_DATA_X_MSB    (0x03)
#define HMC5883L_DATA_X_LSB    (0x04)
#define HMC5883L_DATA_Z_MSB    (0x05)
#define HMC5883L_DATA_Z_LSB    (0x06)
#define HMC5883L_DATA_Y_MSB    (0x07)
#define HMC5883L_DATA_Y_LSB    (0x08)
#define HMC5883L_STATUS        (0x09)
#define HMC5883L_ID_A          (0x0A)
#define HMC5883L_ID_B          (0x0B)
#define HMC5883L_ID_C          (0x0C)
#define HMCTimeDelay 					 (0.002) // changed name from MAGTimeDelay

uint8_t READ_MAG(uint8_t address);
void Init_MAG(void);
void WRITE_MAG(uint8_t address,uint8_t data);
int16_t readSensor(uint8_t regH, uint8_t regL);
void readMAGSensors(int16_t* Mag);
void PrintMAGSensors(void);
void WhoAmIread(void);

#endif

