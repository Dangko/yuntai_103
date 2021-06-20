/**
  ******************************************************************************
  * �ļ���          : M2006.c
  * �ļ�����        : ����������ݽ���
  * ����ʱ��        : 2021.6.13
  * ����            : ������
  ******************************************************************************
  *								�ļ�����     								   *
  ******************************************************************************
  *	
	* ���ļ����ڶ�canͨѶ�Ľ������ݽ��н��룬��ȡ�����ǰ�����Ϣ
	*
  ******************************************************************************
  */
	
#include "M2006.h"

#define ROTER_RANGE	8192

int flag_can_t1=0;
int flag_can_t2=0;
float angle_temp;
int count1=0;
float angle_offset=0;
/** 
* @brief ??��o�쪺?��s???�e??��
* @param None
* @retval None
* @TODO None
*/
void SendMotor()
{

		CanTransmit_12(&hcan, Yuntai.Motor[0].output,Yuntai.Motor[1].output);
}

/**
* @brief ID?1~2��?��H??�e��?
* @param ID?1~2���U??��?�y?��
* @retval None
*/
void CanTransmit_12(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq)
{
	CAN_TxHeaderTypeDef TxMessage;
		
	TxMessage.DLC=0x08;
	TxMessage.StdId=0x200;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;

	uint8_t TxData[8]={0};
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)cm1_iq;
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)cm2_iq; 
	flag_can_t2++;
	uint32_t txmailbox;
	if(HAL_CAN_AddTxMessage(hcanx,&TxMessage,TxData,&txmailbox  )!=HAL_OK)
	{
		flag_can_t1++;
		 Error_Handler();       //�p�GCAN�H��?�e��???�J���`?
	}
}

	
	/**
* @brief ���u?��H����ID??��??��?�u�ѪR
* @param ?��ID?
* @retval None
*/
void CanDataReceive(int motor_index)
{
	switch(motor_index)
	{
		case CAN_Yuntai_MOTOR1_ID:{
			if(Yuntai.Motor[0].msg_cnt++ < 50 ){
				get_rotor_offset(&Yuntai.Motor[0]);
			}
			else{
			CanDataDecoder(&Yuntai.Motor[0]);    
			}
			break;
		}
		case CAN_Yuntai_MOTOR2_ID:{
			if(Yuntai.Motor[1].msg_cnt++ < 50 ){
				get_rotor_offset(&Yuntai.Motor[1]);
			}
			else{
			CanDataDecoder(&Yuntai.Motor[1]);    
			}
			break;
		}
		default:;
	}
}


/**
* @brief CAN�q�H?�󪺤�??�u���^�ѪR��?
* @param ?��?�u?���^
* @retval None
*/
int delta_sum = 0,omega;
void CanDataDecoder(PID_t *motor)
{
	//��s��m
	motor->fdbPos[LAST] = motor->fdbPos[NOW];
	motor->fdbPos[NOW] = CanReceiveData[0]<<8|CanReceiveData[1];
	//��s�t��
	motor->fdb = CanReceiveData[2]<<8|CanReceiveData[3];
	motor->fdb_Vol = CanReceiveData[2]<<8|CanReceiveData[3];
	//��s???
	motor->real_pos[LAST] = motor->real_pos[NOW];

	/* ??��t�פ�?�ȥ�?��?�㫬???����?�㫬 */
	if(motor->fdb > 32768)
	{
		motor->fdb -= 65536;
	}
	 /* ?���?��s */
	if(motor->fdbPos[NOW] - motor->fdbPos[LAST] > ROTER_RANGE / 2)
	{
		motor->round --;
	}
	else if(motor -> fdbPos[NOW] - motor->fdbPos[LAST] < -(ROTER_RANGE/2))
	{
		motor->round ++;
	}
	motor->round_wheel = (float)(motor->round * 187.0f) / 3591;
	motor->real_pos[NOW] = motor->fdbPos[NOW] + ROTER_RANGE * motor->round - motor->offsetPos;
	

	motor ->real_angle = (float)(motor->real_pos[NOW]) * 1.2207f*0.001f;// 1/8192*360*187/3519
	if(count1<=2000) count1+=1;
	if(count1<=1000) angle_offset=motor ->real_angle; 
	angle_temp = motor ->real_angle-angle_offset;
	while(angle_temp>182) angle_temp = angle_temp-180;
	while(angle_temp<-182) angle_temp = angle_temp+180;
	motor->real_angle_360=angle_temp;
	int delta = motor->real_pos[NOW] - motor->real_pos[LAST];
	
	motor->DeltaPosbuf[motor->PosIndex++]=delta;
	
	if(motor->PosIndex == 30){
		delta_sum = 0;
		motor->PosIndex = 0;
		for(int i=0;i<28;i++){
			delta_sum += motor->DeltaPosbuf[i];
		}
		omega = delta_sum		* 0.081729;				// sum/28times/1ms/8192/ * 360 *187 / 3591
	}
	motor->real_w =omega;
	
}

void get_rotor_offset(PID_t *ptr)
{
	ptr->real_pos[NOW] = (uint16_t)CanReceiveData[0]<<8|CanReceiveData[1];
	ptr->offsetPos= ptr->real_pos[NOW];
}


	
	
	
/**
* @brief CAN�~???����l��
* @param can?���^
* @retval None
*/
HAL_StatusTypeDef CanFilterInit(CAN_HandleTypeDef* can)
{
  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
	if(can == &hcan)
	{
		sFilterConfig.FilterBank = 0;
	}

	
  if(HAL_CAN_ConfigFilter(can, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(can) != HAL_OK)
  {
    Error_Handler();
  }
	
  if (HAL_CAN_ActivateNotification(can, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

	return HAL_OK;
}



