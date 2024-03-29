#ifndef IMU_H
#define IMU_H
#include "MKL25Z4.h"

// #define IMI_ADR default is b110100(A0,0 for us)x(R/W) 
#define IMU_ADDR_W (0xD0)
#define IMU_ADDR_R (0xD1)

#define SMPRT_DIV       (0x19)
#define CONFIG           (0x1A)
#define GYRO_CONFIG      (0x1B)
#define ACCEL_CONFIG     (0x1C)
#define FIFO_EN          (0x23)
#define INT_PIN_CFG      (0x37)
#define INT_ENABLE       (0x38)
#define INT_STATUS       (0x3A)
#define ACCEL_XOUT_H     (0x3B)
#define ACCEL_XOUT_L     (0x3C)
#define ACCEL_YOUT_H     (0x3D)
#define ACCEL_YOUT_L     (0x3E)
#define ACCEL_ZOUT_H     (0x3F)
#define ACCEL_ZOUT_L     (0x40)
#define TEMP_OUT_H       (0x41)
#define TEMP_OUT_L       (0x42)
#define GYRO_XOUT_H      (0x43)
#define GYRO_XOUT_L      (0x44)
#define GYRO_YOUT_H      (0x45)
#define GYRO_YOUT_L      (0x46)
#define GYRO_ZOUT_H      (0x47)
#define GYRO_ZOUT_L      (0x48)
#define PWR_MGMT_1       (0x6B)
#define PWR_MGMT_2       (0x6C)
#define WHO_AM_I         (0x75)

uint8_t READ_IMU(uint8_t address);
void Init_IMU(void);
void WRITE_IMU(uint8_t address,uint8_t data);
int16_t readIMUSensor(uint8_t regH, uint8_t regL);
void readIMUSensors(int16_t* Accel,int16_t* Gyro);
void calibrateIMU(int16_t* Aoffset,int16_t* Goffset);
//void RollPitchYaw(void);
void PrintIMUSensors(void);

#endif
