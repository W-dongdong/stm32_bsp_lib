#ifndef __DM3519_LIB_H
#define __DM3519_LIB_H

#include "can_drv.h"
#include <string.h> // This is for memcpy function

typedef enum
{
    disable = 0,
    enable  = 1
} MotorState;

class DM3519{
private:
	CAN_HandleTypeDef* m_hcan;

	uint8_t  m_Error;		// Error flag
	uint16_t m_Pos;			// Encoder
	int16_t  m_Vel;			// Velocity
	uint16_t m_Torque;		// Torque
	uint8_t  m_T_MOS;		// Average MOS temperature
	uint8_t  m_T_Rotor;    
	float	 m_redRatio;	// Reduction Ratio
                       
	
public:
	uint8_t  m_ID;			// Controller ID
	float	 m_SetSpeed;	// Setted target speed of output shaft (rad / s)
	float 	 m_SetPos;		// Setted target position of output shaft (degree)

	DM3519(CAN_HandleTypeDef* hcan, uint16_t ID);

	uint8_t ParseFeedback(CanMsg* RxMsg);
	uint8_t SetMotorState(MotorState State);
	uint8_t SpeedMode(float target_rads);
	uint8_t PosSpeedMode(float Pos, float Speed);
};

#endif
