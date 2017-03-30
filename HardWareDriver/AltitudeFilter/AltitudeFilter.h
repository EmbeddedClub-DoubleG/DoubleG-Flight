#ifndef __ALT_FILT_H
#define __ALT_FILT_H

extern float ALT_Update_Interval;
extern float Filter_Altitude_D;
extern float Filter_Altitude;
extern volatile float evaluateAltitude;
extern float FiltAlt_debug_flag;
void Get_Filter_Altitude(void);

#endif
