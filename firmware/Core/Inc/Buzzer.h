#ifndef _Buzzer_H_
#define _Buzzer_H_

#include <stdbool.h>
#include <stdint.h> 

void BuzzerSingleBeep(uint16_t Duration);
void BuzzerShortBeep(void);
void BuzzerLongBeep(void);
void BuzzerUpdate(void);

#endif
