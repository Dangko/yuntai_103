#ifndef ROS_UART_H
#define ROS_UART_H

#include "main.h"
#include "Yuntai.h"

void RxBuffer_Decode(uint8_t* RxBuffer);
void TxBuffer_Package(uint8_t* TxBuffer,float pos1,float pos2);

extern uint8_t Receive_Data[20];
extern uint8_t Transmit_Data[8];

#endif