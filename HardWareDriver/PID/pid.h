#ifndef PID_H_
#define PID_H_

#define PID_TRUE  0xff
#define PID_FALSE 0x00



struct Quad_PID{
	float target;  // 目标值
	float current; // 当前值
	float merror;
	float last_error;//xiang：上一次的（目标-测量）
	float pre_error;//xiang：这个是我自己加的，用于增量式pid，表示上上一次的（目标-测量）
	float Integrator;	//当前积分值
	float deriv;
	float last_deriv;
	float iLimit;
	float Kp;	   //比例 
	float Ki;	   //积分
	float Kd;	   //微分

	unsigned char Lowpass_EN;
	float outP;         //< proportional output (debugging)
  	float outI;         //< integral output (debugging)
  	float outD;         //< derivative output (debugging)
	float PID_out;   //当前PID 的输出
};

#define DEFAULT_PID_INTEGRATION_LIMIT  100.0

void pidInit(struct Quad_PID* pid, const float kp,
             const float ki, const float kd);
float pidUpdate(struct Quad_PID* pid, float measured,float dt);
float IncreasingPID(struct Quad_PID *pid, float err); //xiang:增量式pid，我自己写的
float pidUpdate_err(struct Quad_PID* pid,float err, float dt);
void pidSetIntegralLimit(struct Quad_PID* pid, float limit);
void pidSetError(struct Quad_PID* pid, float err);
void pidReset(struct Quad_PID* pid);
void pidSetTarget(struct Quad_PID* pid, float target);
void pidSetKp(struct Quad_PID* pid, float kp);
void pidSetKi(struct Quad_PID* pid, float ki);
void pidSetKd(struct Quad_PID* pid, float kd);
void pidSet(struct Quad_PID* pid, float kp,float ki,float kd);
void pidSetMeasured(struct Quad_PID* pid, float measured);
void pidSetTarget_Measure(struct Quad_PID* pid, float target, float measured);
void pidSetLowPassEnable(struct Quad_PID* pid);
#endif
