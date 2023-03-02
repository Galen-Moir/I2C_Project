/* global variables */
#include "queue.h"
#include "globals.h"

unsigned char new_GPS_data=0;

float cur_lat;
float cur_lon;

unsigned long cur_time=0l;
unsigned long cur_date=0l;
float cur_ground_spd = 0.0, cur_course_made_good = 0.0, cur_mag_var = 0.0;
unsigned char GPS_fix_quality=0, GPS_num_sats=0;

float cur_depth=0.0;
float cur_water_spd = 0.0, cur_bearing = 0.0, cur_voltage = 0.1;

volatile int GPS_sentence_avail=0;

/* global mode variables */
uint8_t record_active = FALSE;

// Instrumentation
volatile unsigned long tick_ctr=0;
unsigned long NMEA_checksum_errors=0;
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
