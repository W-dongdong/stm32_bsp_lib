#ifndef __M3508_LIB_H
#define __M3508_LIB_H

#include "can_drv.h"
#include "pid_drv.h"

class M3508{
private:
	uint8_t  m_cascade_frq; 	// Cascade calculation frequency
	CAN_HandleTypeDef* m_hcan;
	uint8_t  m_ID;				// Speed Controller ID(1~4 for 0x200, 5~8 for 0x1FF)
	float	 m_redRatio;		// Reduction Ratio
	int16_t m_encoder_offset;	// Zero point offset for position calibration

	int16_t SpeedModeCalculation(void);
    int16_t PosSpeedModeCalculation(void);
	static  M3508*  MsgAssign(CanMsg* RxMsg);
	static  M3508* Motor_Regester[8];

public:
	static uint8_t SendGroup(CAN_HandleTypeDef* hcan, uint16_t identifier);
	static uint8_t ControlLoopUpdate(CanMsg* RxMsg);

	uint16_t m_Encoder;		// Encoder
	int16_t  m_Vel;			// Velocity
	float 	 m_Torque_Curr;	// Torque current
	uint8_t  m_Temp;		// Temperature of motor(int or uint?)
	int32_t  m_turns;		// Rotate turns
	uint16_t m_last_encoder;// Last value of encoder
	int32_t  m_abs_Pos;		// Total position
                               
	uint8_t  m_State;		// Offline or online
	uint8_t  m_Mode;		// Control mode of motor

	PID 	m_speed_pid;
	PID 	m_pos_pid;

	/*These are some advance parameter*/
	float 	m_StiffnessRate;
	float	m_RecoveryLimit;
	
	

	M3508(CAN_HandleTypeDef* hcan, uint8_t ID);

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
};


#endif
