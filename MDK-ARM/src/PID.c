/**
  ******************************************************************************
  * 文件名          : PID.c
  * 文件描述        : PID计算
  * 创建时间        : 2021.6.13
  * 作者            : 陈凌栩
  ******************************************************************************
  *								文件描述     								   *
  ******************************************************************************
  *	
	* 本文件用于计算云台电机的输出电流
	*
  ******************************************************************************
  */
	
	
#include "PID.h"

void Caculate_Pos()
{
	Pos_Caculate(&Yuntai.Motor[0],&Yuntai.Pos[0]);
	Pos_Caculate(&Yuntai.Motor[1],&Yuntai.Pos[1]);
	PID_Caculate(&Yuntai.Motor[0]);
	PID_Caculate(&Yuntai.Motor[1]);
}

void Caculate_Vel()
{
	PID_Caculate(&Yuntai.Motor[0]);
	PID_Caculate(&Yuntai.Motor[1]);
}

void PID_Caculate(PID_t *pid)
{
		pid->cur_error = pid->ref - pid->fdb;
		pid->output += pid->KP * (pid->cur_error - pid->error[1])  + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
		pid->error[0] = pid->error[1]; 
		pid->error[1] = pid->ref - pid->fdb;
		/*设定输出上限*/
		if(pid->output > pid->outputMax) pid->output = pid->outputMax;
		if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
		if(pid->output<=50&&pid->output>=0) pid->output=0;
	  if(pid->output>=-50&&pid->output<=0) pid->output=0;
}

void Pos_Caculate(PID_t *pid,PosPid_t *pos)
{
		while(pos ->ref>180) pos->ref = pos ->ref-180;
		while(pos ->ref<-180) pos->ref = pos ->ref+180;
		pos->cur_error = pos->ref - pid->real_angle_360;
	  pos->sum_error+=pos->cur_error;
		pos->output = pos->KP * pos->cur_error+pos->KI * pos->sum_error;
		pos->error[0] = pos->error[1]; 
		pos->error[1] = pos->ref - pid->real_angle_360;
		/*设定输出上限*/
		if(pos->output > pos->outputMax) pos->output = pos->outputMax;
		if(pos->output < -pos->outputMax) pos->output = -pos->outputMax;
	  pid->ref = pos->output;
}




