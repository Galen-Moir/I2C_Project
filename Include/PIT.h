#ifndef PIT_H
#define PIT_H

extern char tenth;
extern char sec;
extern char tensec;
extern char hundredsec;


void PIT_init(void);
void PIT_IQRHandler(void);
void Start_PIT(void);
void Stop_PIT(void);

#endif
