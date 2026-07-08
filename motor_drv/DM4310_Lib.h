/**
  ******************************************************************************
  * @file    DM4310_Lib.h
  * @brief   DM4310 Motor Driver Library - Header
  ******************************************************************************
  */

/*
 * File Name        : DM4310_Lib.h
 * Description      : DM4310 motor driver library using CANDevice abstraction.
 *                    Supports Speed / PosSpeed / MIT control modes.
 *                    Each motor owns its TxMsg — mode functions prepare it,
 *                    SendGroup sends all motors on a bus.
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-08
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __DM4310_LIB_H
#define __DM4310_LIB_H

#include "can_drv.h"

class DM4310{
private:
	CANDevice* m_CAN;
	uint8_t   m_ID;              // Motor CAN ID
	uint8_t   m_MST_ID;          // Master ID (for feedback matching)
	uint8_t   m_arr_idx;         // Array index on this bus (creation order)
	float     m_redRatio;        // Reduction Ratio (default 1.0)

	// Static registration per bus (creation-order array)
	static const uint8_t MAX_MOTOR_PER_BUS = 8;
	static DM4310* MotorRegister[CANDevice::MAX_INSTANCES][MAX_MOTOR_PER_BUS];
	static uint8_t  MotorCount[CANDevice::MAX_INSTANCES];

	// Linear mapping helpers (Float ↔ Signed N-bit Integer)
	static int16_t FloatToSInt(float val, float min, float max, uint8_t bits);
	static float   SIntToFloat(int16_t val, float min, float max, uint8_t bits);

public:
	// ========== TxMsg (each motor owns its frame) ==========
	CanMsg   m_TxMsg;            // Prepared control frame
	CanMsg   m_RxMsg;            // Last received feedback
	uint8_t  m_RxFlag;

	// ========== Feedback ==========
	uint8_t  m_Error;            // Error code from feedback
	float    m_Pos;              // Position (rad, motor shaft)
	float    m_Vel;              // Velocity (rad/s, motor shaft)
	float    m_Torque;           // Torque (N·m)
	uint8_t  m_T_MOS;            // MOS temperature (°C)

	// ========== State ==========
	uint8_t  m_Mode;             // 0=Idle, 1=Speed, 2=PosSpeed, 3=MIT

	// ========== Per-instance range limits ==========
	float    m_PMAX;            // Position max  (rad),   default 12.5, MIN = -PMAX
	float    m_VMAX;            // Velocity max  (rad/s), default 30.0, MIN = -VMAX
	float    m_TMAX;            // Torque max    (N.m),   default 10.0, MIN = -TMAX
	float    m_KPMAX;           // Kp max        (MIT),   default 500
	float    m_KDMAX;           // Kd max        (MIT),   default 5
	float    m_TFFMAX;          // T_ff max      (MIT),   default 2,   MIN = -TFFMAX

	DM4310(CANDevice* can, uint8_t ID, uint8_t MST_ID);

	// ========== Feedback ==========
	uint8_t ParseFeedback(CanMsg *RxMsg);

	// ========== Mode setting (targets + TxMsg encoding) ==========
	void SpeedMode(float speed);
	void PosSpeedMode(float pos, float speed_limit);
	void MITMode(float pos, float vel, float kp, float kd, float ff);

	// ========== Configuration ==========
	void Range_Config(float PMAX, float VMAX, float TMAX);

	// ========== Send ==========
	uint8_t SendControl(void);       // Send this motor's m_TxMsg

	// ========== Static ==========
	static uint8_t ControlLoopUpdate(CANDevice* can);
	static uint8_t SendGroup(CANDevice* can);    // Send all motors on a bus
};

#endif
