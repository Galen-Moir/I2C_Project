#ifndef QMC5883L_H
#define QMC5883L_H
#include "MKL25Z4.h"
// Datasheet https://www.makerhero.com/img/files/download/Datasheet-QMC5883L-1.0%20.pdf

#define QMC5883L_DEVICE_ADDRESS   (0x0D)
#define QMC5883L_READ_ADDR        (0x1B)
#define QMC5883L_WRITE_ADDR       (0x1A)

#define QMC5883L_CHIP_ID          (0x0D)

#define QMC5883L_CONFIG_REG_A     (0x09)
#define QMC5883L_CONFIG_REG_B     (0x0A)
#define QMC5883L_SETRESET_REG     (0x0B)

#define QMC5883L_DATA_X_LSB       (0x00)
#define QMC5883L_DATA_X_MSB       (0x01)
#define QMC5883L_DATA_Y_LSB       (0x02)
#define QMC5883L_DATA_Y_MSB       (0x03)
#define QMC5883L_DATA_Z_LSB       (0x04)
#define QMC5883L_DATA_Z_MSB       (0x05)

#define QMC5883L_STATUS_REG       (0x06)
#define QMC5883L_TEMP_LSB         (0x07)
#define QMC5883L_TEMP_MSB         (0x08) 
#define QMCTimeDelay 						  (0.002) // copied HMC time

#define CALIBRATION_SAMPLES 100
#define CALIBRATION_INTERVAL_MS 10
/*
 5 minutes of calibration gave the following values for this particular sensor
 X: Min: -7352 Max: 4782
 Y: Min:-6750 Max: 3955
 Z: Min:-5250 Max:5047
 
 Offset X:-1285,Y:1397.5,Z:-101.5
 AVG:X:6067 Y:5352.5 Z:5148.5, Total:5522.66
 Scale: X:0.91 Y:1.03179 Z:1.0726
*/

void Init_QMC(void);
void WRITE_QMC(uint8_t address,uint8_t data);
void READ_QMC(uint8_t address);
void readQMCSensors();
void calibrateMagnetometer();
void readCalQMCSensors();
float magnetic_bearingGPS(float latA, float longA, float latB, float longB);
float Compass_Bearing();
void compare_Bearing();

#endif

