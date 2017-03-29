#ifndef __ALT_FILT_H
#define __ALT_FILT_H

#include "stm32f4xx.h"

extern float ALT_Update_Interval;
extern float Filter_Altitude_D;
extern float Filter_Altitude;
extern volatile float evaluateAltitude;

void Get_Filter_Altitude(void);

#endif
