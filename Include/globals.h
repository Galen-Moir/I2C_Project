/* globals.h */
#ifndef GLOBALS_H
#define GLOBALS_H

#include "queue.h"
#include "nmea.h"

/** GPS data has been updated */
extern unsigned char new_GPS_data;
/** Current depth information in feet */
extern float cur_depth;
/** Current latitude, in concatenated degrees and minutes */
extern float cur_lat;
/** Current longitude, in concatenated degrees and minutes */
extern float cur_lon;
/** Current time, UTC */
extern unsigned long cur_time;
/** Current date in London! */
extern unsigned long cur_date;
/** Current speed over ground (includes drift, current, etc.) */
extern float cur_ground_spd;
/** Current true course over ground (includes drift, current, etc.) */
extern float cur_course_made_good; 
/** Current magnetic variation */
extern float cur_mag_var;
/** Current bearing of craft */
extern float cur_bearing;
/** Current speed of craft through water, in knots */
extern float cur_water_spd;
/** Current battery voltage */
extern float cur_voltage;

extern uint8_t record_active;

/** GPS fix quality. 0 = none, 1 = GPS, 2 = DGPS */
extern unsigned char GPS_fix_quality;

/** Number of GPS satellites used to determine fix. */
extern unsigned char GPS_num_sats; 

/** Count var requesting GPS sentence processing, set by UART 0 RX ISR */
extern volatile int GPS_sentence_avail;

/** Number of clock ticks scheduler has had */
extern volatile unsigned long tick_ctr;

/** Number of NMEA messages with CRC errors */
extern unsigned long NMEA_checksum_errors;


#endif





// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
