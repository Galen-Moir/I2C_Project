/* nmea.h */

#ifndef NMEA_H
#define NMEA_H

#define NEW_QUEUE_TEST 0

#define TRUE (1)
#define FALSE (0)

#define ENABLE_NMEA_SENTENCE_TESTING 0
#define ENABLE_BAD_NMEA_SENTENCE_TESTING 0

#define USE_OLD_FLASH_CODE 0

// #include "misc.h"
#include "globals.h"
#include <stdint.h>
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** Maximum size of an NMEA sentence */
#define NMEA_SENTENCE_SIZE (96)

/** Data following but not including the $ is checksummed */
#define CSF_START_SYMBOL '$'
/** Data up to but not including the * is checksummed */
#define CSF_END_SYMBOL '*'

#define INPUT (0)
#define OUTPUT (1)

#define FAIL(m) Handle_Sys_Failure(m)

enum display_modes {STATUS1, STATUS2, CHART, MONITOR}; 


#define ENABLE_AUTO_RECORD (0)

#define NMEA_BAUD (4800l)

#define VERIFY_CHECKSUM (0)

// #define USE_RMC    0

typedef enum display_modes DISPMODE_E;

#define DISP_WIDTH (20)
#define BUF_SIZE (DISP_WIDTH+1)
#define LONG_BUF_SIZE (21)

// warning: only works for 0 <= a < 2m-1
#define Q_MODULO(a, m) (a >= m ? a - m : a)

#define TIME_COUNTER_PERIOD (120)  // record a time stamp every this many GPS data points

#define REC_VOLTAGE_PERIOD (120)    // record battery voltage  every this many Sonar data points

#define MAX(a,b)     ((a) > (b))? (a) : (b)
#define MIN(a,b)     ((a) < (b))? (a) : (b)

#define XON           '\021' // octal!
#define XOFF          '\023' // octal!

#define Q_NONE (0)
#define Q_GPS  (1)
#define Q_DGPS (2)


#define TERM_BUF_SIZE 10

/* States */
enum gps_states {GPS_START_STATE, GPS_T1_STATE, GPS_T2_STATE,
		 GPS_S1_STATE, GPS_S2_STATE, GPS_S3_STATE,
		 GPS_SS_STATE,
		 GPS_C1_STATE, GPS_C2_STATE};
typedef enum gps_states GPS_STATES_E;

enum sonar_states {SONAR_START_STATE, 
		   SONAR_TS_STATE, // talker/sentence ID
		   SONAR_SS_STATE, // save sentence body
		   SONAR_C1_STATE, // checksum 1
		   SONAR_C2_STATE, // checksum 2
};
typedef enum sonar_states SONAR_STATES_E;

/** Advance to the next flash page if there are fewer than this many
    bytes remaining in this page */
#define FLASH_WRAP_MARGIN 30

/* Open Issues: 
- method for storing incoming data.  A circular buffer allows efficent
  storage but processing becomes a hassle.  I could copy the data into
  a string for processing, but this would slow things down.  Is there
  a fast way to detect if there's a complete NMEA message in the circ.
  buffer?  Probably just wait to receive another $.

When a new sentence arrives, the $ start character is recognized by
the appropriate UART RX ISR, which could ask the scheduler to run the task
Process_NMEA_Sentence.

Actually, should look for CR/LF, which indicates end of sentence.  Need
to change code to do this.
*/


/** This function removes an NMEA sentence from q anc copies it into buf.
    It returns true if successful, or false if any errors have occurred 
    (message too long, checksum error, or q going empty).
*/
uint8_t Extract_NMEA_Sentence(char * buf, unsigned buf_size, Q_T * q);


/** This function writes selected SONAR information into the DataFlash.
    @arg buf is a pointer to a long buffer used for forming the data
    record, located in the calling function
    (TASK_Process_NMEA_Sentence) for space reasons.
*/
void Save_SONAR_Record(char * buf);

/** This function writes selected GPS information into the DataFlash.
    @arg buf is a pointer to a long buffer used for forming the data
    record, located in the calling function
    (TASK_Process_NMEA_Sentence) for space reasons.
*/
void Save_GPS_Record(char * buf);

/** If we're in record mode and the dataflash is not ready, return
    without processing anything, so that the next time around we'll be
    ready to write to flash. Otherwise, process the latest command
    from the NMEA interface. This is a periodic task which checks
    queues and processes one message at a time as needed, alternating
    between looking at the Sonar and GPS queues.  Look for a DBT or
    GLL message, ignore others.

    Updates the current depth, lat, lon, etc.

    If recording is enabled, this function will form text records as
    needed and then write them to the flash buffer.  If the buffer
    gets close to full (within the largest record set size (total
    records possible from one message) of the end), fill the remainder
    of the buffer with whitespace and program the page.


*/
void TASK_Process_NMEA_Sentence(void);

uint8_t Process_NMEA_DBT(char * buffer);
uint8_t Process_NMEA_GLL(char * buffer);

/** Task which loads queues with fake GPS and Sonar data. */
void TASK_Sim_NMEA(void);

void Test_NMEA_Decoding(void);

void Test_NMEA_FSMs(void);

/* Get latest depth information from sonar.  Sonar information
arrives in a string.  The string is stored character by character in a
SonarBuf.  The UART receive character ISR fills this string. When this
string is complete, the ISR calls a function to parse it and put the
result into global variable CurDepth.  Data sharing problems are
avoided because only the ISR writes to CurDepth. But what if the ISR
interrupts a Get_Depth call?  This won't happen because...  [insert
solution here] */

/** FSM for filtering out unwanted GPS NMEA messages */
void GPS_NMEA_FSM(unsigned char ch, Q_T* q);

/** FSM for filtering out unwanted SONAR NMEA messages */
void SONAR_NMEA_FSM(unsigned char ch, Q_T* q);

#endif
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
