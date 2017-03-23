
#include "pid.h"

// low pass filter:
// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
const float  lowpass_filter = 7.9577e-3;



void pidInit(struct Quad_PID* pid, float kp, float ki, float kd)
{
  pid->merror = 0;
  pid->last_error = 0;
  pid->pre_error=0;//xiang:pre_error是我自己加的东西，用于增量式pid
  pid->Integrator = 0;
  pid->deriv = 0;
  pid->target = 0;
  pid->last_deriv = 0;
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  
  pid->Lowpass_EN = PID_FALSE;   //低通滤波	禁止
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;	//积分限制 xiang:避免积分饱和
}

float pidUpdate(struct Quad_PID *pid, float measured, float dt)
{
  float output;
  pid->current = measured;
  pid->merror = pid->target - measured;

  //I  积分
  if (pid->Ki != 0)
  {
    pid->Integrator += (pid->Ki * pid->merror) * dt;
    if (pid->Integrator > pid->iLimit)
    {
      pid->Integrator = pid->iLimit;
    }
    else if (pid->Integrator < -pid->iLimit)
    {
      pid->Integrator = -pid->iLimit;
    }
  }
  //D 微分
  pid->deriv = (pid->merror - pid->last_error) / dt;
  if (pid->Lowpass_EN != PID_FALSE)
  { //低通滤波。截止频率20hz  xiang:全局看看，把(dt / (lowpass_filter + dt))当成一个参数就可以发现这其实就是一个简单的一阶低通滤波算法
    pid->deriv = pid->last_deriv +
                 (dt / (lowpass_filter + dt)) * (pid->deriv - pid->last_deriv);
  }

  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;

  pid->PID_out = output = pid->outP +
                          pid->outI +
                          pid->outD;

  pid->last_error = pid->merror;
  //xiang:查找了整个项目，没发现last_deriv出现在赋值号左边，所以加了这行代码试试
  pid->last_deriv = pid->deriv;
  return output;
}
/**************************实现函数********************************************
  *函数原型:	float IncreasingPID(struct Quad_PID* pid,float err)
  *功　　能:  增量式pid
  输入：err：当前（目标值-测量值）
  返回：返回的是增量
  xiang：这个函数是我写的
*******************************************************************************/
float IncreasingPID(struct Quad_PID* pid,float err)
{
  float output;
  pid->merror = err;



  pid->outP = pid->Kp * (pid->merror-pid->last_error);
  pid->outI = pid->Ki * pid->merror;
  pid->outD = pid->Kd * (pid->merror-2*(pid->last_error)+pid->pre_error);

  pid->PID_out = output = pid->outP + pid->outI + pid->outD;
  
  pid->pre_error=pid->last_error;
  pid->last_error=pid->merror;
  return output;
}

float pidUpdate_err(struct Quad_PID *pid, float err, float dt)
{
  float output;
  pid->merror = err;
  //I
  pid->Integrator += (pid->Ki * pid->merror) * dt;
  if (pid->Integrator > pid->iLimit)
  {
    pid->Integrator = pid->iLimit;
  }
  else if (pid->Integrator < -pid->iLimit)
  {
    pid->Integrator = -pid->iLimit;
  }
  //D
  pid->deriv = (pid->merror - pid->last_error) / dt;
  if (pid->Lowpass_EN != PID_FALSE)
  { //低通滤波。截止频率20hz
    pid->deriv = pid->last_deriv +
                 (dt / (lowpass_filter + dt)) * (pid->deriv - pid->last_deriv);
  }
  //P
  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;

  pid->PID_out = output = pid->outP +
                          pid->outI +
                          pid->outD;

  pid->last_error = pid->merror;
  //xiang:查找了整个项目，没发现last_deriv出现在赋值号左边，所以加了这行代码试试
  pid->last_deriv = pid->deriv;
  return output;
}

void pidSetError(struct Quad_PID* pid, float err)
{
  pid->merror = err;
}

void pidSetIntegralLimit(struct Quad_PID* pid, float limit)
{
  pid->iLimit = limit;
}

void pidReset(struct Quad_PID* pid)
{
  pid->merror = 0;
  pid->last_error = 0;
  pid->pre_error=0;//xiang:这行是我自己加的，用于增量式pid
  pid->Integrator = 0;
  pid->deriv = 0;
  pid->last_deriv=0;//xiang:这行是我加的，我觉得需要加
}


void pidSetTarget(struct Quad_PID* pid, float target)
{
  pid->target = target;
}

void pidSetTarget_Measure(struct Quad_PID* pid, float target, float measured)
{
  pid->target = target;
  pid->current = measured;
  pid->merror = pid->target - measured;
}

void pidSetKp(struct Quad_PID* pid, float kp)
{
  pid->Kp = kp;
}

void pidSetKi(struct Quad_PID* pid, float ki)
{
  pid->Ki = ki;
}

void pidSetKd(struct Quad_PID* pid, float kd)
{
  pid->Kd = kd;
}

void pidSet(struct Quad_PID* pid, float kp,float ki,float kd)
{
  pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

void pidSetMeasured(struct Quad_PID* pid, float measured)
{
  pid->current = measured;
}

void pidSetLowPassEnable(struct Quad_PID* pid)
{
	pid->Lowpass_EN = PID_TRUE;	
}


//------------------End of File----------------------------
