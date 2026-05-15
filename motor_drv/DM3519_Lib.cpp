#include "DM3519_Lib.h"

#define		PI		3.1415926535f

DM3519::DM3519(CAN_HandleTypeDef* hcan, uint16_t ID) 
    : m_hcan(hcan), m_Error(0), m_Pos(0), m_Vel(0), m_Torque(0), 
	  m_T_MOS(0), m_T_Rotor(0), m_redRatio(3591.0f/187.0f),
	  m_ID(ID), m_SetSpeed(0), m_SetPos(0)
	{
	}

uint8_t DM3519::ParseFeedback(CanMsg* RxMsg)
{
	if (RxMsg == NULL)
	{
		return 0;
	}
	
	uint8_t Rx_ID = RxMsg->Data[0] & 0x0F;
	
	if (Rx_ID != m_ID)//This jugdement need be modified!!!
	{
		return 0;
	}
	else
	{
		m_Error = (RxMsg->Data[0] >> 4) & 0x0F;
		m_Pos = (RxMsg->Data[1] << 8) | RxMsg->Data[2];//待处理
		m_Vel = (int16_t)(((RxMsg->Data[3] << 4) | ((RxMsg->Data[4] >> 4) & 0x0F)) - 0x7FF);//返回最后一位是小数，比如105即10.5,正负代表方向
		m_Torque = ((RxMsg->Data[4] & 0xF) << 8) |  RxMsg->Data[5];//待处理
		m_T_MOS = RxMsg->Data[6];
		m_T_Rotor = RxMsg->Data[7];
		return 1;
	}
}

/**
  * @brief  Set DM3519 State ENABLE or DISABLE
  * @param  State: ENABLE, or DISABLE
  * @retval return 1 if success, else return 0
  */
uint8_t DM3519::SetMotorState(MotorState State)
{
	CanMsg TxMsg;
	TxMsg.ID = m_ID;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 8;
	TxMsg.Data[0] = 0xFF;
	TxMsg.Data[1] = 0xFF;
	TxMsg.Data[2] = 0xFF;
	TxMsg.Data[3] = 0xFF;
	TxMsg.Data[4] = 0xFF;
	TxMsg.Data[5] = 0xFF;
	TxMsg.Data[6] = 0xFF;
	if (State == enable){
		TxMsg.Data[7] = 0xFC;
	}else{
		TxMsg.Data[7] = 0xFD;
	}
	return CAN_Send_Msg(m_hcan, &TxMsg, 5);
}

uint8_t DM3519::SpeedMode(float target_rads)
{	
	CanMsg TxMsg;
	int32_t Speed;
	
	m_SetSpeed = target_rads * m_redRatio;	// Update speed of motor
	
	TxMsg.ID = (uint32_t)(m_ID + 0x200);
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 4;
	
	memcpy(&Speed, &m_SetSpeed, 4);// WTF???
	TxMsg.Data[0] = (Speed >> 0) & 0xFF;
    TxMsg.Data[1] = (Speed >> 8) & 0xFF;
    TxMsg.Data[2] = (Speed >> 16) & 0xFF;
    TxMsg.Data[3] = (Speed >> 24) & 0xFF;
	
	// Return 1 only if transmission is successfully
	return CAN_Send_Msg(m_hcan, &TxMsg, 5);
}

uint8_t DM3519::PosSpeedMode(float target_Pos, float target_Speed)
{
	CanMsg TxMsg;
	int32_t Speed;
	int32_t Pos;
	
	m_SetPos = target_Pos * m_redRatio * (PI/180.0f);	// Update position of motor
	m_SetSpeed = target_Speed * m_redRatio;				// Update speed of motor
	
	memcpy(&Speed, &m_SetSpeed, 4);
	memcpy(&Pos, &m_SetPos, 4);
	
	TxMsg.ID = m_ID + 0x100;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 8;
	
	TxMsg.Data[0] = (Pos >> 0) & 0xFF;
    TxMsg.Data[1] = (Pos >> 8) & 0xFF;
    TxMsg.Data[2] = (Pos >> 16) & 0xFF;
    TxMsg.Data[3] = (Pos >> 24) & 0xFF;
	
	TxMsg.Data[4] = (Speed >> 0) & 0xFF;
    TxMsg.Data[5] = (Speed >> 8) & 0xFF;
    TxMsg.Data[6] = (Speed >> 16) & 0xFF;
    TxMsg.Data[7] = (Speed >> 24) & 0xFF; 
	
	return CAN_Send_Msg(m_hcan, &TxMsg, 5);
}
