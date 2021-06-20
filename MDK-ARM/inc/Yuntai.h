#ifndef DIFF_WHEEL_H
#define DIFF_WHEEL_H

#include "main.h"

#define NOW 1
#define LAST 0

/**
* @brief 底盘工作状态枚举类型
* @param None
* @retval None
*/
typedef enum 
{
	Stop_Mode,
	Vel_Mode,	
	Pos_Mode
}WorkState_t;

/**
* @brief PID结构体
* @param None
* @retval None
*/
typedef struct
{
	int32_t msg_cnt;
	float real_w;
	int32_t DeltaPosbuf[30];
	int8_t PosIndex;
	float round_wheel;
	int32_t fdbPos[2];
	int32_t offsetPos;
	
	int32_t real_pos[2];
	int fdb_Vol;
	
	float real_angle;
	float real_angle_360;
	int16_t round;
	
	float KP;
	float KI;
	float KD;
	int fdb;
	int error[2];
	
	int ref;
	
	int cur_error;
	int output;
	int outputMax;
	
	int fdb_v;
	
}PID_t;

typedef struct 
{
	float KP;
	float KI;
	float KD;
	float cur_error;
	float sum_error;
	int output;
	int outputMax;
	int error[2];
	float ref;
}PosPid_t;

/**
* @brief 遥控器接收数据解析结构体
* @param None
* @retval None
*/
typedef struct 
{
    int w_t;
    int n_t;	
}Target_t;

/**
* @brief 底盘全部参数结构体
* @param None
* @retval None
*/
typedef struct
{
	Target_t Target;
	PID_t Motor[2];
	PosPid_t Pos[2];
	WorkState_t WorkState;
}Yuntai_t;

void Yuntai_Init();

extern Yuntai_t Yuntai;
extern uint8_t UsartReceiveRemote[18];               
extern uint8_t CanReceiveData[8];                      



#endif