/**
  ******************************************************************************
  * @file    DM4310_Lib.cpp
  * @brief   DM4310 Motor Driver Library - Implementation
  ******************************************************************************
  */

/*
 * File Name        : DM4310_Lib.cpp
 * Description      : DM4310 motor driver library using CANDevice abstraction.
 *                    Supports Speed / PosSpeed / MIT control modes.
 *                    Each motor owns its TxMsg -- mode functions prepare it,
 *                    SendGroup sends all motors on a bus.
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : DM4310_Lib.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-08
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */


#include "DM4310_Lib.h"
#include <string.h>


DM4310* DM4310::MotorRegister[CANDevice::MAX_INSTANCES][MAX_MOTOR_PER_BUS] = {};
uint8_t  DM4310::MotorCount[CANDevice::MAX_INSTANCES] = {};


int16_t DM4310::FloatToSInt(float val, float min, float max, uint8_t bits)
{
	if (val < min) val = min;
	if (val > max) val = max;

	uint32_t range = (1ul << bits) - 1;          // e.g. 65535 for 16-bit

	float result = (val - min) / (max - min) * (float)range;

	if (result > (float)range) result = (float)range;
	if (result < 0.0f)         result = 0.0f;

	return (int16_t)result;                      // truncate
}

float DM4310::SIntToFloat(int16_t val, float min, float max, uint8_t bits)
{
	uint32_t range  = (1ul << bits) - 1;
	int32_t  offset = (int32_t)(1ul << (bits - 1));   // int32_t avoids overflow
	int32_t  sv     = (int32_t)val;                    // sign-extend

	return (float)(sv + offset) / (float)range * (max - min) + min;
}


DM4310::DM4310(CANDevice* can, uint8_t ID, uint8_t MST_ID)
	: m_CAN(can)
	, m_ID(ID)
	, m_MST_ID(MST_ID)
	, m_arr_idx(0)
	, m_redRatio(1.0f)
	, m_RxFlag(0)
	, m_Error(0)
	, m_Pos(0.0f)
	, m_Vel(0.0f)
	, m_Torque(0.0f)
	, m_T_MOS(0)
	, m_Mode(0)          /* Idle */
	, m_PMAX(12.5f)
	, m_VMAX(30.0f)
	, m_TMAX(10.0f)
{
	if (can == NULL) return;

	// Clear TxMsg
	m_TxMsg.ID   = 0;
	m_TxMsg.IDE  = CAN_ID_STD;
	m_TxMsg.RTR  = CAN_RTR_DATA;
	m_TxMsg.DLC  = 8;
	for (uint8_t i = 0; i < 8; i++){
		m_TxMsg.Data[i] = 0;
	}

	// Clear RxMsg
	m_RxMsg.ID   = 0;
	m_RxMsg.IDE  = 0;
	m_RxMsg.RTR  = 0;
	m_RxMsg.DLC  = 0;
	for (uint8_t i = 0; i < 8; i++){
		m_RxMsg.Data[i] = 0;
	}

	// Register on bus (creation order)
	uint8_t bus_idx = can->m_bus_idx;
	if (bus_idx < CANDevice::MAX_INSTANCES && MotorCount[bus_idx] < MAX_MOTOR_PER_BUS)
	{
		m_arr_idx = MotorCount[bus_idx];
		MotorRegister[bus_idx][MotorCount[bus_idx]++] = this;
	}
}


/**
  * @brief  Parse DM4310 motor CAN feedback message.
  * @param  RxMsg: Pointer to received CAN message (8 bytes)
  * @retval 1: Parse success, 0: Parse failed (MST_ID mismatch / NULL / wrong DLC)
  *
  * Feedback frame layout (8 bytes, CAN ID = motor_id):
  *   Byte 0  [7:4] MST_ID,  [3:0] ERR
  *   Byte 1  Motor ID
  *   Byte 2  POS[15:8]
  *   Byte 3  POS[7:0]           -> 16-bit signed -> linear map -> rad
  *   Byte 4  VEL[11:4]
  *   Byte 5  [7:4] VEL[3:0],  [3:0] T[11:8]
  *   Byte 6  T[7:0]             -> 12-bit signed each -> linear map
  *   Byte 7  T_MOS              -> 8-bit unsigned -> degC
  */
uint8_t DM4310::ParseFeedback(CanMsg *RxMsg)
{
	if (RxMsg == NULL || RxMsg->DLC != 8){
		return 0;
	}

	// Match by MST_ID in Data[0] bits [7:4]
	uint8_t rx_mst_id = (RxMsg->Data[0] >> 4) & 0x0F;
	if (rx_mst_id != m_MST_ID){
		return 0;
	}

	// --- Byte 0: [MST_ID | ERR] ---
	m_Error = RxMsg->Data[0] & 0x0F;

	// --- Byte 2-3: Position (16-bit signed) ---
	int16_t raw_pos = (int16_t)(((uint16_t)RxMsg->Data[2] << 8) | RxMsg->Data[3]);

	// --- Byte 4-5: Velocity (12-bit signed) ---
	//   Data[4] = VEL[11:4],  Data[5][7:4] = VEL[3:0]
	uint16_t u_vel = ((uint16_t)RxMsg->Data[4] << 4) | ((RxMsg->Data[5] >> 4) & 0x0F);
	int16_t raw_vel;
	if (u_vel & 0x800){
		raw_vel = (int16_t)(u_vel | 0xF000);
	} else {
		raw_vel = (int16_t)u_vel;
	}

	// --- Byte 5-6: Torque (12-bit signed) ---
	//   Data[5][3:0] = T[11:8],  Data[6] = T[7:0]
	uint16_t u_torque = ((uint16_t)(RxMsg->Data[5] & 0x0F) << 8) | RxMsg->Data[6];
	int16_t raw_torque;
	if (u_torque & 0x800){
		raw_torque = (int16_t)(u_torque | 0xF000);
	} else {
		raw_torque = (int16_t)u_torque;
	}

	// --- Byte 7: MOS temperature ---
	m_T_MOS = RxMsg->Data[7];

	// --- Linear mapping: raw -> physical (rad, rad/s, N.m) ---
	m_Pos    = SIntToFloat(raw_pos,    -m_PMAX, m_PMAX, 16);
	m_Vel    = SIntToFloat(raw_vel,    -m_VMAX, m_VMAX, 12);
	m_Torque = SIntToFloat(raw_torque, -m_TMAX, m_TMAX, 12);

	return 1;
}


/**
  * @brief  Set Speed control mode.
  * @param  speed: Target speed (rad/s), range -30 ~ +30
  *
  * Control frame: ID = 0x200 + m_ID, DLC = 8
  *   Byte 0-3: v_des (32-bit IEEE 754 float, rad/s)
  *   Byte 4-7: 0
  */
void DM4310::SpeedMode(float speed)
{
	m_Mode = 1;

	m_TxMsg.ID  = 0x200 + m_ID;
	m_TxMsg.IDE = CAN_ID_STD;
	m_TxMsg.RTR = CAN_RTR_DATA;
	m_TxMsg.DLC = 4;

	memcpy(&m_TxMsg.Data[0], &speed, 4);

	m_TxMsg.Data[4] = 0;
	m_TxMsg.Data[5] = 0;
	m_TxMsg.Data[6] = 0;
	m_TxMsg.Data[7] = 0;
}

/**
  * @brief  Set Position-Speed cascade control mode.
  * @param  pos:         Target position (rad)
  * @param  speed_limit: Speed limit (rad/s)
  *
  * Control frame: ID = 0x100 + m_ID, DLC = 8
  *   Byte 0-3: p_des (32-bit IEEE 754 float, little-endian)
  *   Byte 4-7: v_des (32-bit IEEE 754 float, little-endian)
  */
void DM4310::PosSpeedMode(float pos, float speed_limit)
{
	m_Mode = 2;

	m_TxMsg.ID  = 0x100 + m_ID;
	m_TxMsg.IDE = CAN_ID_STD;
	m_TxMsg.RTR = CAN_RTR_DATA;
	m_TxMsg.DLC = 8;

	memcpy(&m_TxMsg.Data[0], &pos, 4);
	memcpy(&m_TxMsg.Data[4], &speed_limit, 4);
}

/**
  * @brief  Set MIT (impedance) control mode.
  * @param  pos: Target position (rad), range -PMAX ~ +PMAX
  * @param  vel: Target velocity (rad/s), range -VMAX ~ +VMAX
  * @param  kp:  Stiffness (12-bit direct, no range mapping)
  * @param  kd:  Damping   (12-bit direct, no range mapping)
  * @param  ff:  Feedforward torque (12-bit direct, no range mapping)
  *
  * Control frame: ID = 0x00 + m_ID, DLC = 8
  *   Byte 0-1: p_des   (16-bit unsigned BE)
  *   Byte 2:   v_des[11:4]
  *   Byte 3:   v_des[3:0] << 4  |  Kp[11:8]
  *   Byte 4:   Kp[7:0]
  *   Byte 5:   Kd[11:4]
  *   Byte 6:   Kd[3:0] << 4  |  T_ff[11:8]
  *   Byte 7:   T_ff[7:0]
  */
void DM4310::MITMode(float pos, float vel, float kp, float kd, float ff)
{
	m_Mode = 3;

	m_TxMsg.ID  = 0x00 + m_ID;
	m_TxMsg.IDE = CAN_ID_STD;
	m_TxMsg.RTR = CAN_RTR_DATA;
	m_TxMsg.DLC = 8;

	// Bytes 0-1: p_des (16-bit unsigned, big-endian)
	int16_t raw_pos = FloatToSInt(pos, -m_PMAX, m_PMAX, 16);
	m_TxMsg.Data[0] = (raw_pos >> 8) & 0xFF;
	m_TxMsg.Data[1] = raw_pos & 0xFF;

	// v_des, Kp, Kd, T_ff: 12-bit each, nibble-packed (6 bytes for 4 values)
	int16_t raw_vel = FloatToSInt(vel, -m_VMAX, m_VMAX, 12);
	int16_t raw_kp  = FloatToSInt(kp, 0, 500, 12);
	int16_t raw_kd  = FloatToSInt(kd, 0, 5, 12);
	int16_t raw_ff  = FloatToSInt(ff, -m_TMAX, m_TMAX, 12);

	m_TxMsg.Data[2] = (raw_vel >> 4) & 0xFF;                        // v_des[11:4]
	m_TxMsg.Data[3] = ((raw_vel & 0x0F) << 4) | ((raw_kp >> 8) & 0x0F); // v_des[3:0] | Kp[11:8]
	m_TxMsg.Data[4] = raw_kp & 0xFF;                                 // Kp[7:0]
	m_TxMsg.Data[5] = (raw_kd >> 4) & 0xFF;                         // Kd[11:4]
	m_TxMsg.Data[6] = ((raw_kd & 0x0F) << 4) | ((raw_ff >> 8) & 0x0F); // Kd[3:0] | T_ff[11:8]
	m_TxMsg.Data[7] = raw_ff & 0xFF;                                 // T_ff[7:0]
}


void DM4310::Range_Config(float PMAX, float VMAX, float TMAX)
{
	if (PMAX > 0.0f) m_PMAX = PMAX;
	if (VMAX > 0.0f) m_VMAX = VMAX;
	if (TMAX > 0.0f) m_TMAX = TMAX;
}

uint8_t DM4310::SendControl(void)
{
	if (m_CAN == NULL || m_Mode == 0){
		return 0;
	}
	return m_CAN->Send_Msg(&m_TxMsg, 5);
}



/**
  * @brief  Parse CAN feedback for all motors on a bus (if-check by MST_ID).
  * @param  can: CANDevice that received the message (reads can->m_RxMsg).
  * @retval 1 if a motor matched, 0 otherwise.
  *
  * Call in main loop when can->m_RxFlag is set.
  * NOT ISR-safe (float math inside ParseFeedback).
  */
uint8_t DM4310::ControlLoopUpdate(CANDevice* can)
{
	if (can == NULL) return 0;

	uint8_t bus_idx = can->m_bus_idx;
	if (bus_idx >= CANDevice::MAX_INSTANCES) return 0;

	CanMsg* RxMsg = &can->m_RxMsg;

	for (uint8_t i = 0; i < MotorCount[bus_idx]; i++)
	{
		DM4310* motor = MotorRegister[bus_idx][i];
		if (motor != NULL)
		{
			if (motor->ParseFeedback(RxMsg))
			{
				motor->m_RxMsg  = can->m_RxMsg;
				motor->m_RxFlag = 1;
				return 1;
			}
		}
	}
	return 0;
}

/**
  * @brief  Send control frames for all registered motors on a CAN bus.
  * @param  can: CANDevice instance (identifies the bus via m_bus_idx).
  * @retval Number of motors successfully sent.
  *
  * Call periodically (e.g. 1 kHz timer) to keep motors active.
  * Motors in Idle mode (m_Mode == 0) are skipped.
  */
uint8_t DM4310::SendGroup(CANDevice* can)
{
	if (can == NULL) return 0;

	uint8_t bus_idx = can->m_bus_idx;
	if (bus_idx >= CANDevice::MAX_INSTANCES) return 0;

	uint8_t sent = 0;
	for (uint8_t i = 0; i < MotorCount[bus_idx]; i++)
	{
		DM4310* motor = MotorRegister[bus_idx][i];
		if (motor != NULL && motor->m_Mode != 0)
		{
			if (motor->SendControl()){
				sent++;
			}
		}
	}
	return sent;
}
