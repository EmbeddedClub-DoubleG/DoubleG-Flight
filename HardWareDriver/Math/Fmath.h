#ifndef __FMATH_H
#define __FMATH_H


#define PitchRollEXP  50   //0.5
#define PitchRollRate 100  //1.0

#define ThrMid   0
#define Thr_EXP  40	   

float Math_fConstrain(float value, float min, float max);
int16_t Math_Constrain(int16_t value,int16_t min,int16_t max);
int16_t Math_abs(int16_t value);
int16_t Math_min(int16_t value1,int16_t value2);
int16_t Math_max(int16_t value1,int16_t value2);
void Math_init_EXP(void);
int16_t Math_ThrEXP(int16_t RCThr);
int16_t Math_AngelEXP(int16_t in);

#endif

//------------------End of File----------------------------
