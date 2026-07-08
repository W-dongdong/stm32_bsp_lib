/**
  ******************************************************************************
  * @file    M3508_Lib.h
  * @brief   M3508 Motor Driver Library - Header
  ******************************************************************************
  */

/*
 * File Name        : M3508_Lib.h
 * Description      : M3508 motor driver library using CANDevice abstraction.
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can_drv.h, pid_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __M3508_LIB_H
#define __M3508_LIB_H

#include "can_drv.h"
#include "pid_drv.h"

class M3508{
private:
	uint8_t  m_cascade_frq; 	// Cascade calculation frequency
	CANDevice* m_CAN;
	uint8_t  m_ID;				// Speed Controller ID(1~8 per bus)
	float	 m_redRatio;		// Reduction Ratio
	int16_t m_encoder_offset;	// Zero point offset for position calibration

	int16_t SpeedModeCalculation(void);
	int16_t PosSpeedModeCalculation(void);
	int16_t TorqueModeCalculation(void);
	int16_t MITModeCalculation(void);
	static const uint8_t MAX_MOTOR_PER_BUS = 8;
	static M3508* MsgAssign(CANDevice* can);
	static M3508* MotorRegester[CANDevice::MAX_INSTANCES][MAX_MOTOR_PER_BUS];

public:
	static uint8_t SendGroup(CANDevice* can, uint16_t identifier);
	static uint8_t ControlLoopUpdate(CANDevice* can);

	// ========== Feedback ==========
	uint16_t m_Encoder;		// Encoder
	int16_t  m_Vel;			// Velocity
	float 	 m_Torque_Curr;	// Torque current
	uint8_t  m_Temp;		// Temperature
	int32_t  m_turns;		// Rotate turns
	uint16_t m_last_encoder;// Last value of encoder
	int32_t  m_abs_Pos;		// Total position

	// ========== State ==========
	uint8_t  m_State;		// Offline or online
	uint8_t  m_Mode;		// Control mode

	// ========== PID ==========
	PID 	m_speed_pid;
	PID 	m_pos_pid;

	// ========== Advanced ==========
	float 	m_StiffnessRate;
	float	m_RecoveryLimit;

	// ========== Torque mode ==========
	struct {
		float Kt;		// Torque constant (Nm/A)
		float target;	// Target torque (Nm)
	} m_Torque;

	// ========== MIT mode ==========
	struct {
		float angle;	// Target angle on output shaft (deg)
		float vel;		// Target velocity on output shaft (rpm)
		float kp;		// Stiffness (Nm/deg)
		float kd;		// Damping (Nm/rpm)
		float ff;		// Feedforward torque (Nm)
	} m_MIT;

	M3508(CANDevice* can, uint8_t ID);

	void 	PID_Config(const PID& Pos_Loop, const PID& Speed_Loop);
	void	FeedForward_Config(float flexibility, float smoothness);
	void	ActiveRecovery_Config(float stiffnessRate, float recoveryLimit);
	uint8_t ParseFeedback(CanMsg *RxMsg);
	void 	SpeedMode(int16_t target_speed);
	void 	PosSpeedMode(float target_Angel, uint16_t Speed_Limit);
	int16_t pid_calc(void);
	uint8_t MsgAppend(int16_t Calcu_result);
	void 	SetZeroPoint(void);
	float   ActiveRecoveryCalculation(void);

	// MIT / Torque mode
	void	TorqueConstant_Config(float Kt);
	void	TorqueMode(float target_torque);
	void	MITMode(float target_angle, float target_vel, float kp, float kd, float ff);
};


#endif