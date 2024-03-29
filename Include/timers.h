#ifndef TIMERS_H
#define TIMERS_H
#include "MKL25Z4.h"

#define LCD_UPDATE_PERIOD (100)

uint32_t UTC_TO_BIN_SEC(unsigned UTC);
void Init_RTC_SEC_INT(void);
void Init_PIT(unsigned period);
void Start_PIT(void);
void Stop_PIT(void);

void Init_PWM(void);
void Set_PWM_Values(uint16_t perc1, uint16_t perc2);

extern volatile unsigned RTC_Second_signal;
extern volatile unsigned PIT_interrupt_counter;
extern volatile unsigned LCD_update_requested;

#endif
// *******************************ARM University Program Copyright � ARM Ltd 2013*************************************   
