#include "Mission.h"
#include "PID.h"
#include "Yuntai.h"
#include "M2006.h"

void Mission_Start()
{
	switch (Yuntai.WorkState)
	{
		case Stop_Mode:
			Yuntai.Pos[0].ref=0;
			Yuntai.Pos[1].ref=0;
			Caculate_Vel();
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
			CanTransmit_12(&hcan, Yuntai.Motor[0].output, Yuntai.Motor[1].output);
			break;
		case Vel_Mode:
			Caculate_Vel();
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			CanTransmit_12(&hcan, Yuntai.Motor[0].output, Yuntai.Motor[1].output);
			break;
		case Pos_Mode:
			Caculate_Pos();
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			CanTransmit_12(&hcan, Yuntai.Motor[0].output, Yuntai.Motor[1].output);
			break;
	}
}

