#ifndef KIN_SOL_H
#define KIN_SOL_H

#include "Yuntai.h"

#define PI 3.1415926

void PID_Caculate(PID_t *pid);
void Pos_Caculate(PID_t *pid,PosPid_t *pos);
void Caculate_Pos();
void Caculate_Vel();


#endif