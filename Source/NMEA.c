/* nmea.c */
#include "NMEA.h"
#include "UART.h"

#if ((ENABLE_NMEA_SENTENCE_TESTING || SIM_NMEA_INPUTS))
_far const char nmea_gps[20][] = {
  "$GPGSV,3,2,11,14,13,236,00,15,37,314,38,18,58,329,36,21,81,229,43*78\r\n",
  "$GPGGA,120127,3537.857,N,07847.430,W,1,05,3.5,118.2,M,-33.7,M,,*77\r\n",
  "$GPGSA,A,3,,,09,,,15,18,21,22,,,,4.8,3.5,3.2*3E\r\n",
  "$GPGSV,3,1,11,03,08,307,00,06,08,199,00,09,49,129,48,10,02,095,00*76\r\n",
  "$GPGSV,3,3,11,22,31,297,43,26,31,044,32,29,22,047,00,,,,*4F\r\n",
  "$PGRME,14.0,M,12.8,M,18.9,M*10\r\n",
  "$GPGLL,3537.857,N,07847.430,W,120128,A*33\r\n",
  "$PGRMZ,388,f,3*18\r\n",
  "$PGRMM,WGS 84*06\r\n",
  "$GPBOD,,T,,M,,*47\r\n",
  "$GPRTE,1,1,c,0*07\r\n",
  "$PSLIB,,,K*23\r\n",
  "$PSLIB,,,J*22\r\n",
  "$GPRMC,120129,A,3537.859,N,07847.429,W,000.0,226.7,100904,008.5,W*7A\r\n",
  "$GPRMC,120130,A,3537.860,N,07847.430,W,000.0,226.7,100904,008.6,W*73\r\n",
  "$GPRMB,A,,,,,,,,,,,,V*71\r\n",
  ""};

_far const char nmea_sonar[9][] = {
  "$YXXDR,R,0.0,l,PORT FUEL,R,0.0,l,STARBOARD FUEL,U,12.4,V,BATTERY*4F\r\n",
  "$SDDBT,0.0,f,0.0,M,0.0,F*06\r\n",
  "$SDDPT,0.0,0.0,2.0*57\r\n",
  "$PTTKV,0.0,,,49.6,49.6,72.3,F*11\r\n",
  "$PTTKD,0.0,,B*1F\r\n",
  "$VWVHW,,,,,0.0,N,0.0,K*4D\r\n",
  "$VWMTW,22.4,C*16\r\n",
  "$VWVLW,49.6,N,49.6,N*4C\r\n",
  ""};

#endif

#if ENABLE_BAD_NMEA_SENTENCE_TESTING
/* These strings are used to test for decoding of bad NMEA sentences */
_far const char nmea_gps[20][] = {
  "$GPGSV,3,2,11,14,13,36,00,15,37,314,38,18,58,329,36,21,81,229,43*78\r\n",
  "$GPGGA,120127,3537857,N,07847.430,W,1,05,3.5,118.2,M,-33.7,M,,*77\r\n",
  "$GPGSA,A,3,,,09,,15,18,21,22,,,,4.8,3.5,3.2*3E\r\n",
  "$GPGSV,3,1,11,03,8,307,00,06,08,199,00,09,49,129,48,10,02,095,00*76\r\n",
  "$GPGSV,3,3,11,225,31,297,43,26,31,044,32,29,22,047,00,,,,*4F\r\n",
  "$PGRME,14.0,M,12.28,M,18.9,M*10\r\n",
  "$GPGLL,3537.857,r,07847.430,W,120128,A*33\r\n",
  "$PGRMZ,388,w,3*18\r\n",
  "$PGRMM,WGS-84*06\r\n",
  "$GPBOD,,T,M,,*47\r\n",
  "$GPRTE,1,9,c,0*07\r\n",
  "$PSLIB,,,Ka*23\r\n",
  "$PSLnIB,,,J*22\r\n",
  "$GdPRMC,120129,A,3537.859,N,07847.429,W,000.0,226.7,100904,008.5,W*7A\r\n",
  "$GPsRMB,A,,,,,,,,,,,,V*71\r\n",
  ""};

_far const char nmea_sonar[32][] ={
  /* 0 */ "$YXsXDR,R,0.0,l,PORT FUEL,R,0.0,l,STARBOARD FUEL,U,12.4,V,BATTERY*4F\r\n",
  /* 1 */ "$SDDBT,00.0,f,0.0,M,0.0,F*06\r\n",
  /* 2 */ "$SDDPT,90.0,0.0,2.0*57\r\n",
  /* 3 */ "$PTTKV,0.0,,,49.6,49.6,7.3,F*11\r\n",
  /* 4 */ "$PTTKD,0.0,,B*1B\r\n",                        /* bad CRC - should be 1F */
  /* 5 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 6 */ "$VWVHW,,,,,0.0,N,0.0,K*D\r\n",                /* bad CRC - one character short, should be 4D */
  /* 7 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 8 */ "$VWMTW,22.4,C*161\r\n",                       /* extra character at end of CRC, should be 16. Acceptable error. */
  /* 9 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 10 */ "$VWVLW,49.6,N,49.6,N*4C\n",                   /* missing \r */
  /* 11 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 12 */ "$VWVLW,49.6,N,49.6,N*4C\r",                   /* missing \n */
  /* 13 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 14 */ "$VWVLW,49.6,N,49.6,N*4C\n\r",                 /* order swapped */
  /* 15 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 16 */ "$VWVLW,49.6,N,49.6,N*4C",                     /* missing \r\n */
  /* 17 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 18 */ "$YXXDR,R,0.0,l,PORT FUEL,R,0.0,l,STARBOARD FUEL,U,12.4,V,BATTERY*4F$SDDBT,0.0,f,0.0,M,0.0,F*06\r\n", /* merged messages w/o \r\n */
  /* 19 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 20 */ "$SDDPT,0.0,0.0,2.0*57\r\n$PTTKV,0.0,,,49.6,49.6,72.3,F*11\r\n", /* merged messages with \r\n */
  /* 21 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 22 */ "SDDPT,0.0,0.0,2.0*57\r\n",            	  /* message fragment -- start missing */
  /* 23 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 24 */ "TKV,0.0,,,49.6,49.6,72.3,F*11\r\n",	          /* message fragment -- start missing */
  /* 25 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 26 */ "$PTTKD,0.0,,B",			          /* message fragment -- end missing */
  /* 27 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  /* 28 */ "$VWVHW,,,,,0.0,N,0.0,K*",		          /* message fragment -- end missing */
  /* 29 */ "$VWVLW,49.6,N,49.6,N*4C\r\n",                 /* good message */
  ""};

#endif

void Scan_Float( char * * pp, float * f) {
  char * p = *pp;
  if (*p == ',') {
    p++; /* skip comma */
    *f = 0.0;
  } else {
    *f = atof(p);
    while (*p != ',')
      p++;
    p++; /* skip comma */
  }
  *pp = p;
}

void Scan_Char( char * * pp, char * c) {
  char * p = *pp;
  if (*p == ',') {
    p++; /* skip comma */
    *c = '\0';
  } else {
    *c = *p;
    while (*p != ',')
      p++;
    p++; /* skip comma */
  }
  *pp = p;
}
void Scan_Long( char * * pp, long * l) {
  char * p = *pp;
  if (*p == ',') {
    p++; /* skip comma */
    *l = 0l;
  } else {
    *l = atol(p);
    while (*p != ',')
      p++;
    p++; /* skip comma */
  }
  *pp = p;
}

uint8_t Extract_NMEA_Sentence(char * buf, unsigned buf_size, Q_T * q){
  unsigned char c;
  unsigned num_chars=0;
  unsigned char checksum=0, sent_checksum=0;
  unsigned char sent_checksum_ctr=0; // current nibble of sent checksum
  unsigned char received_full_checksum=0; // flag to indicate if both nibbles of checksum have arrived */
  unsigned char saw_CSF_start=0, saw_CSF_end=0; // flags to indicate whether in checksum data field
  
  do {   /* first advance to the first $ */
    if (Q_Empty(q))
      return FALSE;
    c = Q_Dequeue(q);
  } while (c != '$');
  
  while ((c != '\r') && (c != '\n') && (!received_full_checksum)) {
    if (sent_checksum_ctr) { /* start reconstructing the sent checksum */ 
      sent_checksum_ctr--;
      if (sent_checksum_ctr == 0)
				received_full_checksum = 1;
			sent_checksum <<= 4;
      if ((c >= '0') && (c <= '9'))
				sent_checksum += c - '0';
      else if ((c >= 'A') && (c <= 'F'))
				sent_checksum += c - 'A' + 10;
      else if ((c >= 'a') && (c <= 'f'))
				sent_checksum += c - 'a' + 10;
      else /* corrupt checksum */
				return FALSE;
    }
    
    if (CSF_END_SYMBOL == c) { /* detect end of data to checksum */
      saw_CSF_end = 1;
      sent_checksum_ctr = 2;
    }
    if (saw_CSF_start && !saw_CSF_end) /* update checksum */
      checksum ^= c;
    if (CSF_START_SYMBOL == c) /* detect end of data to checksum */
      saw_CSF_start = 1;
    
    *buf = c;
    buf++;
    num_chars++;
    if (num_chars >= buf_size)
      //      FAIL("ENS: Buffer overflow");
      return FALSE;
    
    if (Q_Empty(q))
      return FALSE; /* shouldn't ever happen! Was a while loop before. */
    c = Q_Dequeue(q);  
  }  
   
  *buf = '\0'; /* terminate string */ 
#if VERIFY_CHECKSUM
  if (checksum != sent_checksum) {
    NMEA_checksum_errors++;
    return FALSE;
  } else 
    return TRUE;
#else
		return TRUE;
#endif
}



uint8_t Process_NMEA_RMC(char * buffer) {
  long time=0l, date=0l;
  float lat=0.0, lon=0.0, ground_spd=0.0, course_made_good=0.0, mag_var=0.0;
  char lat_d=' ', lon_d=' ', valid=' ', mag_var_d=' ';
  
  Scan_Long(&buffer, &time);
  Scan_Char(&buffer, &valid);
  Scan_Float(&buffer, &lat);
  Scan_Char(&buffer, &lat_d);
  Scan_Float(&buffer, &lon);
  Scan_Char(&buffer, &lon_d);
  Scan_Float(&buffer, &ground_spd);
  Scan_Float(&buffer, &course_made_good);
  Scan_Long(&buffer, &date);
  Scan_Float(&buffer, &mag_var);
  Scan_Char(&buffer, &mag_var_d);
  
  cur_time = time;
  cur_date = date;

	new_GPS_data = 1;
  
  if ((valid == 'A') || (valid == 'a')) {
    if ((lat_d=='S') || (lat_d=='s'))
      lat = -lat;
    if ((lon_d=='E') || (lon_d=='e'))
      lon = -lon;
    cur_lat = lat;
    cur_lon = lon;
    cur_date = date;
    cur_ground_spd = ground_spd;
    cur_course_made_good = course_made_good;
    if ((mag_var_d == 'E') || (mag_var_d == 'e'))
      cur_mag_var = -mag_var;
    else
      cur_mag_var = mag_var;
    return TRUE;
  }
  return FALSE;
}

uint8_t Process_NMEA_GGA(char * buffer) {
  long time=0l, date=0l, temp=0l;
  float lat=0.0, lon=0.0;
  char lat_d=' ', lon_d=' ';
  
  Scan_Long(&buffer, &time);
  //  Scan_Char(&buffer, &valid);
  Scan_Float(&buffer, &lat);
  Scan_Char(&buffer, &lat_d);
  Scan_Float(&buffer, &lon);
  Scan_Char(&buffer, &lon_d);

  // GPS Quality: 0/1/2
  Scan_Long(&buffer, &temp);
	GPS_fix_quality = temp & 0xff;

  // # satellites in use
  Scan_Long(&buffer, &temp);
  GPS_num_sats = temp & 0xff;

  cur_time = time;

  new_GPS_data = 1;

  if ((GPS_fix_quality == Q_GPS) || (GPS_fix_quality == Q_DGPS)) {
    if ((lat_d=='S') || (lat_d=='s'))
      lat = -lat;
    if ((lon_d=='E') || (lon_d=='e'))
      lon = -lon;
    cur_lat = lat;
    cur_lon = lon;
    //    cur_date = date;
    return TRUE;
  }
  return FALSE;
}

uint8_t Process_NMEA_GLL(char * buffer) {
  long time=0l;
  float lat=0.0, lon=0.0;
  char lat_d=' ', lon_d=' ', valid=' ';
  
  Scan_Float(&buffer, &lat);
  Scan_Char(&buffer, &lat_d);
  Scan_Float(&buffer, &lon);
  Scan_Char(&buffer, &lon_d);
  Scan_Long(&buffer, &time);
  if (*buffer != '*') /* make sure validity char is there, and not already at checksum */
    Scan_Char(&buffer, &valid); 
  
  cur_time = time;
  
  if ((valid == 'A') || (valid == 'a')) {
    if ((lat_d=='S') || (lat_d=='s'))
      lat = -lat;
    if ((lon_d=='E') || (lon_d=='e'))
      lon = -lon;
    cur_lat = lat;
    cur_lon = lon;
    return TRUE;
  }
  return FALSE;
}



void TASK_Process_NMEA_Sentence(void) {
  char buffer[NMEA_SENTENCE_SIZE];
  char * p;
    
  if (GPS_sentence_avail) 
	{
		memset(buffer, 0, NMEA_SENTENCE_SIZE); /* erase string */
	  if (Extract_NMEA_Sentence(buffer, NMEA_SENTENCE_SIZE, &RxQ)) {
	    p = buffer;
	    p++; /* discard talker info */
	    p++;
	    p++;

	    if (!strncmp(p, "RMC,", 4)) {
	      Process_NMEA_RMC(p+4);
	    } else if (!strncmp(p, "GGA,", 4)) {
	      Process_NMEA_GGA(p+4);
	    } else if (!strncmp(p, "GLL,", 4)) {
	      Process_NMEA_GLL(p+4);
	    } else {
	      /* ignore unsupported messages */
	    }
	  }
	  GPS_sentence_avail--;
	}
 }
	
#if SIM_NMEA_INPUTS
void TASK_Sim_NMEA(void) {
  static unsigned char g_i = 0, s_i = 0;
  if (submode == MONITOR)
    return; // don't load queues when in monitor mode
  
  Q_Enqueue_String(&GPS_RX_Q, nmea_gps[g_i]);
  GPS_sentence_avail = 1;
  g_i++;
  if (strlen(nmea_gps[g_i]) == 0)
    g_i = 0;
  
  Q_Enqueue_String(&SONAR_RX_Q, nmea_sonar[s_i]);
  sonar_sentence_avail = 1;
  s_i++;
  if (strlen(nmea_sonar[s_i]) == 0)
    s_i = 0;
}
#endif // SIM_NMEA_INPUTS

void Test_NMEA_Decoding(void) {
  /* This function tests the decoding of various NMEA sentences, which
     are stored in ROM. */
  unsigned int i;
#if 0
  // hardwired GPGLL, GPRMC message input!
  i = 13;
  while (nmea_gps[i][0]) {           //  i!
    Q_Enqueue_String(&GPS_RX_Q, nmea_gps[i]); /*  add string to queue */
    GPS_sentence_avail = 1;
    TASK_Process_NMEA_Sentence(); // call twice since alternates
    TASK_Process_NMEA_Sentence();
  } // infinite loop
#endif
  
#if 0
  i = 5;
  while (nmea_sonar[i][0]) {
    Q_Enqueue_String(&SONAR_RX_Q, nmea_sonar[i]); /*  add string to queue */
    sonar_sentence_avail = 1;
    TASK_Process_NMEA_Sentence(); // call twice since alternates
    TASK_Process_NMEA_Sentence();
  } // infinite loop
#endif
  
#if ENABLE_NMEA_SENTENCE_TESTING || ENABLE_BAD_NMEA_SENTENCE_TESTING
  // step through all messages
  i = 0;
  while (nmea_gps[i][0]) {           
    Q_Enqueue_String(&GPS_RX_Q, nmea_gps[i]); /*  add string to queue */
    GPS_sentence_avail = 1;
    TASK_Process_NMEA_Sentence();
    TASK_Process_NMEA_Sentence();
    i++;
  }
  i = 0;
  while (nmea_sonar[i][0]) {
    Q_Enqueue_String(&SONAR_RX_Q, nmea_sonar[i]); /*  add string to queue */
    sonar_sentence_avail = 1;
    TASK_Process_NMEA_Sentence();  // call twice since the task alternates 
    TASK_Process_NMEA_Sentence();
    i++;
  }
#endif // NMEA sentence testing
}

void Test_NMEA_FSMs(void) {
  unsigned msg, ch;
  
#if ENABLE_NMEA_SENTENCE_TESTING 
  // test GPS FSM
  // This should load up the GPS_RX_Q with only GPRMC messages
  for (msg = 0; nmea_gps[msg][0]; msg++) {
    for (ch = 0; nmea_gps[msg][ch]; ch++) {
      GPS_NMEA_FSM(nmea_gps[msg][ch], &GPS_RX_Q); 
    }
  }
  // This shouldn't load any messages (from SONAR) into the  GPS_RX_Q
  for (msg = 0; nmea_sonar[msg][0]; msg++) {
    for (ch = 0; nmea_sonar[msg][ch]; ch++) {
      GPS_NMEA_FSM(nmea_sonar[msg][ch], &GPS_RX_Q); 
    }
  }  
  
  msg = 13;
  for (ch = 0; nmea_gps[msg][ch]; ch++) {
    GPS_NMEA_FSM(nmea_gps[msg][ch], &GPS_RX_Q); 
  }
  msg = 14;
  for (ch = 0; nmea_gps[msg][ch]; ch++) {
    GPS_NMEA_FSM(nmea_gps[msg][ch], &GPS_RX_Q); 
  }
  
  // test sonar FSM
  // This should load up the SONAR_RX_Q with only DBT, VHW and XDR messages
  for (msg = 0; nmea_sonar[msg][0]; msg++) {
    for (ch = 0; nmea_sonar[msg][ch]; ch++) {
      SONAR_NMEA_FSM(nmea_sonar[msg][ch], &SONAR_RX_Q); 
    }
  }
  // This shouldn't load any messages (from GPS) into the  SONAR_RX_Q
  for (msg = 0; nmea_sonar[msg][0]; msg++) {
    for (ch = 0; nmea_sonar[msg][ch]; ch++) {
      SONAR_NMEA_FSM(nmea_gps[msg][ch], &SONAR_RX_Q); 
    }
  }  
  
  //  Now process the messages
  while (1)
    TASK_Process_NMEA_Sentence();
  
#endif
  
}


void GPS_NMEA_FSM(unsigned char ch, Q_T* q) {
  static GPS_STATES_E cur_GPS_state=GPS_START_STATE;
  static char local_GPS_buffer[10];
  static unsigned char index=0;
  unsigned char i;
  
  switch (cur_GPS_state) {
  case GPS_START_STATE: // recognize start of message 
    index = 0;
    if (ch == '$') {
      cur_GPS_state = GPS_T1_STATE;
      local_GPS_buffer[index++] = ch;
    } 
    break;
  case GPS_T1_STATE: 
    if (ch == 'G') {
      cur_GPS_state = GPS_T2_STATE;
      local_GPS_buffer[index++] = ch;      
    }
    else 
      cur_GPS_state = GPS_START_STATE;
    break;
  case GPS_T2_STATE:
    if (ch == 'P') {
      cur_GPS_state = GPS_S1_STATE;
      local_GPS_buffer[index++] = ch;
    }
    else 
      cur_GPS_state = GPS_START_STATE;
    break;
  case GPS_S1_STATE:
    if ((ch == 'G') || (ch == 'R'))			{ // R
      cur_GPS_state = GPS_S2_STATE;
      local_GPS_buffer[index++] = ch;
    }
    else 
      cur_GPS_state = GPS_START_STATE;
    break;
  case GPS_S2_STATE:
    if ((ch == 'G') || (ch == 'M')) { // M
      cur_GPS_state = GPS_S3_STATE;
      local_GPS_buffer[index++] = ch;
    }
    else 
      cur_GPS_state = GPS_START_STATE;
    break;
  case GPS_S3_STATE:
    if ((ch == 'A') || (ch == 'C')) { // C
      cur_GPS_state = GPS_SS_STATE;
      local_GPS_buffer[index++] = ch;
      // enqueue everything so far in this message
      for (i=0; i<index; i++)
		Q_Enqueue(q, local_GPS_buffer[i]);
    }
    else 
      cur_GPS_state = GPS_START_STATE;
    break;
  case GPS_SS_STATE:
    if (ch == '$')
      cur_GPS_state = GPS_START_STATE;
    else {
      Q_Enqueue(q, ch); // enqueue the character
      if (ch == '*')
		cur_GPS_state = GPS_C1_STATE;	// start checksum recognizer
    }
    break;
  case GPS_C1_STATE:
    Q_Enqueue(q, ch);  // save first character of CS
    cur_GPS_state = GPS_C2_STATE;
    break;
  case GPS_C2_STATE:
    Q_Enqueue(q, ch);  // save second character of CS
    // These two characters are added to reduce the number of states required.
    // Extract_NMEA_Sentence expects to see these characters 
    Q_Enqueue(q, '\r'); // save terminator (expected in rest of code)
    Q_Enqueue(q, '\n'); // save terminator (expected in rest of code)
    
    GPS_sentence_avail++;
    cur_GPS_state = GPS_START_STATE;
    break;
  default: 
    break;
  }
}

	
	
	
	

// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
