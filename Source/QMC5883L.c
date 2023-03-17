
#include "timers.h"
#include "MKL25Z4.h"
#include <math.h>
#include "I2C_LCD.h"
#include "IMU.h"
#include "i2c.h"
#include "delay.h"
#include "QMC5883L.h"
#include <stdio.h>
#include <math.h>
#include "globals.h"

int8_t Data[1] ={0};
uint8_t MAGSensorData[6] ={0};
int16_t CombinedData[3] ={0};
float CombinedFloat[3] = {0};
float CombinedCalData[3] ={0};
// Calibration data
float offsetX, offsetY, offsetZ;
float scaleX, scaleY, scaleZ;
float AvgX, AvgY, AvgZ,AvgTotal;
float normX,normY,normZ;
float xMin = 0, yMin = 0, zMin = 0;
float xMax = 0, yMax = 0, zMax = 0;
float MeasuredoffsetX=-3912, MeasuredoffsetY=-2024, MeasuredoffsetZ=101.5;
//Offset X:-1285,Y:1397.5,Z:-101.5
float MeasuredscaleX=0.93431, MeasuredscaleY=1.075626, MeasuredscaleZ=1.0726;
//Scale: X:0.91 Y:1.03179 Z:1.0726
// Raw data read from magnetometer
float rawX, rawY, rawZ;
float MagneticDeclination = -12.82; // Constant for ottawa
#define PI 3.14159265358979323846
#define MAGNETIC_DECLINATION_OTTAWA -12.81
float Xdata[1000], Ydata[1000], Bdata[1000];
float savedBearing;
float SavedCurrentBearing;
float SavedCurrentTargetBearing;


void Init_QMC(void){

	Delay(2); // delay 2ms before powerup for writing to QMC
	WRITE_QMC(QMC5883L_CONFIG_REG_A,0x89);  // 0x81 for normal operation, 0x89 for calibration, over sample 128, 2G range, 10Hz data or 100 hz, continuous mode
	WRITE_QMC(QMC5883L_CONFIG_REG_B,0x41);  // enable pointer rolling, disable interrupts
	WRITE_QMC(QMC5883L_SETRESET_REG,0x01);
	//calibrateMagnetometer();
	Clear_LCD(LCD_ADDR_W);
}

void WRITE_QMC(uint8_t address,uint8_t data){
	
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = QMC5883L_WRITE_ADDR;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C1->D = data;
	I2C_WAIT;
	
	I2C_M_STOP;							/*send stop	*/
}

void READ_QMC(uint8_t address){
	uint8_t buff[40];
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = QMC5883L_WRITE_ADDR;
	I2C_WAIT;
	
	I2C1->D = address;
	I2C_WAIT;
	
	I2C_M_RSTART;
	I2C1->D = QMC5883L_READ_ADDR;
	I2C_WAIT;
	
	I2C_REC;
	NACK;
	
	Data[0] = I2C1->D;					/*dummy read	*/
	I2C_WAIT;								/*wait for completion */
	
	I2C_M_STOP;							/*send stop	*/
	Data[0] = I2C1->D;					/*read data	*/
	sprintf(buff, "reg:%x", Data[0]);
	Print_I2C_LCD_String(LCD_ADDR_W,buff);
	
}

void readQMCSensors()
{
	uint8_t buff[40];
	int i;
	Set_Cursor(LCD_ADDR_W,0,0);
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = QMC5883L_WRITE_ADDR;
	I2C_WAIT;
	
	I2C1->D = QMC5883L_DATA_X_LSB;
	I2C_WAIT;
	
	I2C_M_RSTART;
	I2C1->D = QMC5883L_READ_ADDR;
	I2C_WAIT;
	
	I2C_REC;
	// Read five bytes in repeated mode
	for( i=0; i<5; i++)	{
	MAGSensorData[i] = i2c_repeated_read(0);
	}
	// Read last byte ending repeated mode
	MAGSensorData[i] = i2c_repeated_read(1);
	CombinedData[0] = ((MAGSensorData[1]<<8) | MAGSensorData[0]); 
	CombinedData[1] = ((MAGSensorData[3]<<8) | MAGSensorData[2]);
	CombinedData[2] = ((MAGSensorData[5]<<8) | MAGSensorData[4]);
	CombinedFloat[0] = (float)CombinedData[0];
	CombinedFloat[1] = (float)CombinedData[1];
	CombinedFloat[2] = (float)CombinedData[2];
	sprintf(buff, "X:%.2f Y:%.2f", CombinedFloat[0],CombinedFloat[1]);
	Print_I2C_LCD_String(LCD_ADDR_W,buff);
	Set_Cursor(LCD_ADDR_W,0,1);
	sprintf(buff, "Z:%.2f", CombinedFloat[2]);
	Print_I2C_LCD_String(LCD_ADDR_W,buff);
}

void readCalQMCSensors()
{
	
	uint8_t buff[40];
	int i;
	Delay(1);
	I2C_TRAN;
	I2C_M_START;
	I2C1->D = QMC5883L_WRITE_ADDR;
	I2C_WAIT;
	
	I2C1->D = QMC5883L_DATA_X_LSB;
	I2C_WAIT;
	
	I2C_M_RSTART;
	I2C1->D = QMC5883L_READ_ADDR;
	I2C_WAIT;
	
	I2C_REC;
	// Read five bytes in repeated mode
	for( i=0; i<5; i++)	{
	MAGSensorData[i] = i2c_repeated_read(0);
	}
	// Read last byte ending repeated mode
	MAGSensorData[i] = i2c_repeated_read(1);
	CombinedData[0] = ((MAGSensorData[1]<<8) | MAGSensorData[0]);
	CombinedData[1] = ((MAGSensorData[3]<<8) | MAGSensorData[2]);
	CombinedData[2] = ((MAGSensorData[5]<<8) | MAGSensorData[4]);
	CombinedCalData[0] = (float)CombinedData[0];
	CombinedCalData[1] = (float)CombinedData[1];
	CombinedCalData[2] = (float)CombinedData[2];
	CombinedCalData[0] = ((CombinedCalData[0]-MeasuredoffsetX)*MeasuredscaleX)/16384; // 2^16/4 due to +-2G range
	CombinedCalData[1] = ((CombinedCalData[1]-MeasuredoffsetY)*MeasuredscaleY)/16384;
	CombinedCalData[2] = ((CombinedCalData[2]-MeasuredoffsetZ)*MeasuredscaleZ)/16384;
	//sprintf(buff, "X:%.2f Y:%.2f", CombinedCalData[0],CombinedCalData[1]);
	//Print_I2C_LCD_String(LCD_ADDR_W,buff);
	//Set_Cursor(LCD_ADDR_W,0,1);
	//sprintf(buff, "Z:%f.2", CombinedCalData[2]);
	//Print_I2C_LCD_String(LCD_ADDR_W,buff);
	//Delay(200);
	
}

void calibrateMagnetometer() {
    int i;

		uint8_t buff[40];
    
    // Collect calibration data
    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
				readQMCSensors();
        // Read raw data from magnetometer
        rawX = (float)CombinedData[0];
        rawY = (float)CombinedData[1];
        rawZ = (float)CombinedData[2];
        
        // Update minimum and maximum values
        if (rawX < xMin) {
            xMin = rawX;
        }
        if (rawY < yMin) {
            yMin = rawY;
        }
        if (rawZ < zMin) {
            zMin = rawZ;
        }
        if (rawX > xMax) {
            xMax = rawX;
        }
        if (rawY > yMax) {
            yMax = rawY;
        }
        if (rawZ > zMax) {
            zMax = rawZ;
        }
        
        // Wait for next sample
      Delay(CALIBRATION_INTERVAL_MS);
			Set_Cursor(LCD_ADDR_W,0,0);
			sprintf(buff, "Cal samp:%d", i);
			Print_I2C_LCD_String(LCD_ADDR_W,buff);
    }
    
    // Calculate calibration offsets
    offsetX = (xMax + xMin) / 2.0;
    offsetY = (yMax + yMin) / 2.0;
    offsetZ = (zMax + zMin) / 2.0;
    
    // Calculate calibration scales
		// scale can be (xMax-xMin)/2
		AvgX=(xMax - xMin)/2.0;
		AvgY=(yMax - yMin)/2.0;
		AvgZ=(zMax - zMin)/2.0;
		AvgTotal = (AvgX+AvgY+AvgZ)/3;
		
		scaleX = AvgTotal/AvgX;
		scaleY = AvgTotal/AvgY;
		scaleZ = AvgTotal/AvgZ;
		
}


/*Returns a magnetic compass bearing in degrees 
that can be compared to the magnetometer output 
when given latitude and longitude of two points in degrees*/
//output angle is clockwise from true north

float magnetic_bearingGPS(float latA, float longA, float latB, float longB){
  
	uint32_t m;
	float latARadians = latA * PI / 180;
	float longARadians = longA * PI / 180;
	float latBRadians = latB * PI / 180;
	float longBRadians = longB * PI / 180;

	float x = cos(latBRadians) * sin (longBRadians - longARadians);

	float y = cos(latARadians) * sin(latBRadians) - sin(latARadians) * cos(latBRadians) * cos(longBRadians - longARadians);

	float angle = atan2(x, y) * 180 / PI + MAGNETIC_DECLINATION_OTTAWA;

	
	if (angle >= 0){
				angle = angle;
			}	else {
				angle = angle+360;}
	SavedCurrentTargetBearing = angle;
	return angle;
}


float Compass_Bearing(){ // add int j input for data collection
	uint8_t buff[40];
	float average, angle;
	int i;

	for(i=0;i<10;i++){
	readCalQMCSensors();
	angle = atan2(CombinedCalData[1]*(PI/180),CombinedCalData[0]*(PI/180));
	angle = (1.7899) - angle; // 1.7899 is the 12.5 degrees of magnetic declinaton added to pi/2 if 90 degrees was considered true north
	angle = angle*(180/PI);
	if (angle > 180) {
    angle -= 360;
}
	average = average+angle;
	}
	angle = (average/10);
	//sprintf(buff, "%.2f", angle);
	//Print_I2C_LCD_String(LCD_ADDR_W,buff);
	//Set_Cursor(LCD_ADDR_W,0,1);
	//sprintf(buff, "x:%.2f y:%.2f", CombinedCalData[0],CombinedCalData[1]);
	//Print_I2C_LCD_String(LCD_ADDR_W,buff);
	SavedCurrentBearing = angle;
	//Xdata[j]=CombinedCalData[0] ;
	//Ydata[j]=CombinedCalData[1]; 
	//Bdata[j]=angle;
	return angle;
}

void compare_Bearing(){
	float currentBearing,targetBearing;
	float bearingReformat = 0;
	float difference = 0;
	int printLat,printLon;
	uint8_t buff[40];
	currentBearing = SavedCurrentBearing;
	targetBearing = SavedCurrentTargetBearing;
	
	//Reformat the bearing from the magnetometer so that it matches calculated target bearing (0-360) 
	if (currentBearing < 0){
		bearingReformat = -1*currentBearing;
	} else {
		bearingReformat = 360 - currentBearing;
	}
	
	//Find absolute value of difference between bearings
	difference = bearingReformat - targetBearing;
	
	if (difference<0){
		difference = difference+360;
	}
	if( (difference < 5) || (difference > 355) ){
				Print_I2C_LCD_String(LCD_ADDR_W,"Straight");
	}else if(difference>180){
		right = 1;
		left = 0;
		Print_I2C_LCD_String(LCD_ADDR_W,"Right   ");
	} else {
		right =0;
		left=1;
		Print_I2C_LCD_String(LCD_ADDR_W,"Left    ");
	}
	sprintf(buff, "  %f",difference);
	Print_I2C_LCD_String(LCD_ADDR_W,buff);
//printLat = (int)(cur_lat*10000000.0)-((int)(cur_lat*1000.0)*10000.0);
//printLon = ((int)(cur_lon*10000000.0)-((int)(cur_lon*1000.0)*10000.0));
Set_Cursor(LCD_ADDR_W,0,1);
//sprintf(buff, "%d    %d", printLat,printLon);
sprintf(buff, "%.2f  %.3f", currentBearing,targetBearing);
Print_I2C_LCD_String(LCD_ADDR_W,buff);
}

