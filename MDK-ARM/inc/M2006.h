#ifndef M2006_H
#define M2006_H

#include "main.h"
#include "Yuntai.h"
#include "can.h"

#define CAN_Yuntai_MOTOR1_ID 0x201
#define CAN_Yuntai_MOTOR2_ID 0x202
#define CAN_Yuntai_MOTOR3_ID 0x203
#define CAN_Yuntai_MOTOR4_ID 0x204
#define CAN_Yuntai_MOTOR5_ID 0x205


void SendMotor();
void CanTransmit_12(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq);
void CanDataReceive(int motor_index);
void CanDataDecoder(PID_t *motor);
void get_rotor_offset(PID_t *ptr);
HAL_StatusTypeDef CanFilterInit(CAN_HandleTypeDef* hcan);

extern int count1;

#endif