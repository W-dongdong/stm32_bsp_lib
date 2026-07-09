/**
  ******************************************************************************
  * @file    M3508_Lib.cpp
  * @brief   M3508 Motor Driver Library - Implementation
  ******************************************************************************
  */

/*
 * File Name        : M3508_Lib.cpp
 * Description      : M3508 motor driver library using CANDevice abstraction.
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : M3508_Lib.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */


#include "M3508_Lib.h"

#define	k	20.0f / 16384.0f

// Tx buffers per bus (0x200: motors 1-4, 0x1FF: motors 5-8)
static CanMsg TxGroup1[CANDevice::MAX_INSTANCES] = {
	{0x200, CAN_ID_STD, CAN_RTR_DATA, 8, {0}},
	{0x200, CAN_ID_STD, CAN_RTR_DATA, 8, {0}},
	{0x200, CAN_ID_STD, CAN_RTR_DATA, 8, {0}},
};
static CanMsg TxGroup2[CANDevice::MAX_INSTANCES] = {
	{0x1FF, CAN_ID_STD, CAN_RTR_DATA, 8, {0}},
	{0x1FF, CAN_ID_STD, CAN_RTR_DATA, 8, {0}},
	{0x1FF, CAN_ID_STD, CAN_RTR_DATA, 8, {0}},
};

// 2D register: [bus_index][motor_ID-1]
M3508* M3508::MotorRegester[CANDevice::MAX_INSTANCES][MAX_MOTOR_PER_BUS] = {};

M3508::M3508(CANDevice* can, uint8_t ID)
    : m_cascade_frq(3),
	  m_CAN(can),
      m_ID(ID),
	  m_redRatio(3591.0f/187.0f),
	  m_encoder_offset(-1), // -1 is a sentinel value
      m_Encoder(0),
      m_Vel(0),
      m_Torque_Curr(0.0f),
      m_Temp(0),
      m_turns(0),
	  m_last_encoder(0),
	  m_abs_Pos(0),
      m_State(0),
	  m_Mode(0),	/* No mode */
      m_speed_pid(0.7, 0.01, 0.1, 2000.0f, 15000.0f),
      m_pos_pid(20, 0, 5, 1500.0f, 2000.0f),
	  m_StiffnessRate(0),
	  m_RecoveryLimit(0),
	  m_Kt(0.3f)
{
	m_MIT.angle  = 0;
	m_MIT.vel    = 0;
	m_MIT.kp     = 0;
	m_MIT.kd     = 0;
	m_MIT.ff     = 0;

	if (can == NULL) return;

	uint8_t bus_idx = can->m_bus_idx;
	if (bus_idx < CANDevice::MAX_INSTANCES && m_ID >= 1 && m_ID <= MAX_MOTOR_PER_BUS)
	{
        MotorRegester[bus_idx][m_ID - 1] = this;
    }
}

/**
  * @brief  Initialize parameters of position PID and speed PID
  * @param  Pos_Loop: Position loop PID parameters
  * @param  Speed_Loop: Speed loop PID parameters
  * @retval None
  */
void M3508::PID_Config(const PID& Pos_Loop, const PID& Speed_Loop)
{
	m_pos_pid = Pos_Loop;
	m_speed_pid = Speed_Loop;
}

void M3508::TorqueConstConfig(float Kt)
{
	if (Kt > 0) {
		m_Kt = Kt;
	} else {
		m_Kt = -Kt;
	}
}

void M3508::FeedForward_Config(float flexibility, float smoothness)
{
	m_speed_pid.SetFeedForward(flexibility, smoothness);
}

void M3508::ActiveRecovery_Config(float stiffnessRate, float recoveryLimit)
{
	m_StiffnessRate = stiffnessRate;
	m_RecoveryLimit = recoveryLimit;
}

void M3508::MITMode(float target_angle, float target_vel, float kp, float kd, float ff)
{
	m_MIT.angle = target_angle;
	m_MIT.vel   = target_vel;
	m_MIT.kp    = kp;
	m_MIT.kd    = kd;
	m_MIT.ff    = ff;
	m_Mode = 3; // MIT mode
}

/**
 * @brief Parse M3508 motor CAN feedback message
 * @param RxMsg: Pointer to received CAN message
 * @return 1: Parse success, 0: Parse failed
 */
uint8_t M3508::ParseFeedback(CanMsg *RxMsg)
{
	if (RxMsg == NULL || RxMsg->DLC != 8){
		return 0;
	}
	// Check ID range
	if (RxMsg->ID < 0x201 || RxMsg->ID > 0x208){
		return 0;
	}

	m_Encoder = (RxMsg->Data[0] << 8) | RxMsg->Data[1];				// 0 ~ 8191
	m_Vel 	  = (int16_t)((RxMsg->Data[2] << 8) | RxMsg->Data[3]);	// rmp
	m_Temp 	  = RxMsg->Data[6];										// degree

	int16_t Raw_Curr = (RxMsg->Data[4] << 8) | RxMsg->Data[5];
	m_Torque_Curr = k * Raw_Curr;									// -20 ~ 20

	// Self-calibration
    if (m_encoder_offset < 0) // when power on
	{
        m_encoder_offset = m_Encoder; // Record the initial position as zero
        m_last_encoder   = m_Encoder; // prevent jump on 2nd frame
        m_turns          = 0;
        m_abs_Pos        = 0;
        return 1;
    }

	// Record turns it rotate
	if (m_last_encoder - m_Encoder > 4096){
		m_turns ++;
	}else if (m_last_encoder - m_Encoder < -4096){
		m_turns --;
	}

	m_abs_Pos = (m_turns * 8192 + m_Encoder) - m_encoder_offset;
	m_last_encoder = m_Encoder;	// Update the encoder

	return 1;
}


void M3508::SpeedMode(int16_t target_speed)
{
	m_speed_pid.setTarget(target_speed);
	m_Mode = 1; // 1 is SpeedMode
}


int16_t M3508::SpeedModeCalculation(void)
{
	m_speed_pid.setMeasure(m_Vel);

	int16_t result = (int16_t)m_speed_pid.calculate();

	//Prevent overflow
	if (result >= 16384){
		result = 16384;
	}else if(result <= -16384){
		result = -16384;
	}

	return result;
}


void M3508::PosSpeedMode(float target_Angel, uint16_t Speed_Limit)
{
	float target_counts = (target_Angel * m_redRatio * 8192.0f) / 360.0f;
	m_pos_pid.setTarget(target_counts);	// Set angle target(degree, output shaft)
	m_pos_pid.m_output_limit = Speed_Limit; // Set Speed limitation
	m_Mode = 2; // Set mode as PosSpeed mode
}

float M3508::ActiveRecoveryCalculation(void)
{
    float Pos_error = m_pos_pid.m_error;
    float Vel_error = m_speed_pid.m_error;
    float coupled_error = Pos_error * Vel_error;

    float x = coupled_error * m_StiffnessRate;
    float cubic_x = x * x * x;

	float result = 0;

	if (m_speed_pid.m_error > 150){
		result = std::fabsf(m_RecoveryLimit * std::tanhf(cubic_x));
	}else if (m_speed_pid.m_error < -150){
		result = -std::fabsf(m_RecoveryLimit * std::tanhf(cubic_x));
	}else{
		result = 0;
	}
    return result;
}


int16_t M3508::PosSpeedModeCalculation(void)
{
	m_speed_pid.setMeasure(m_Vel);

	if(m_cascade_frq >= 3)
	{
		m_cascade_frq = 0;
		m_pos_pid.setMeasure(m_abs_Pos);

		m_speed_pid.setTarget(m_pos_pid.calculate());
	}

	m_cascade_frq ++;

	float pid_val = m_speed_pid.calculate();
	float ActiveRecovery = ActiveRecoveryCalculation();
	float out_val = pid_val + ActiveRecovery;

	// Prevent overflow
	if (out_val > 16384.0f){
        out_val = 16384.0f;
    } else if (out_val < -16384.0f){
        out_val = -16384.0f;
    }

	return (int16_t)out_val;
}


int16_t M3508::MITModeCalculation(void)
{
	// Encoder pulses -> output shaft degrees (single conversion, no double round-trip)
	const float PULSE_TO_DEG = 360.0f / (8192.0f * m_redRatio);
	float pos_deg = (float)m_abs_Pos * PULSE_TO_DEG;
	float pos_error_deg = m_MIT.angle - pos_deg;

	// Velocity at output shaft (rpm), kd unit: Nm/rpm
	float vel_output_shaft = (float)m_Vel / m_redRatio;
	float vel_error = m_MIT.vel - vel_output_shaft;

	// Impedance: torque = Kp * pos_err + Kd * vel_err + FF
	float torque_Nm = m_MIT.kp * pos_error_deg + m_MIT.kd * vel_error + m_MIT.ff;

	// Torque(Nm) -> Current(A) -> CAN raw value
	const float SCALE = 16384.0f / 20.0f;
	float current_A = torque_Nm / m_Kt;
	float can_raw = current_A * SCALE;

	if(can_raw > 16384.0f){
		can_raw = 16384.0f;
	}else if(can_raw < -16384.0f){
		can_raw = -16384.0f;
	}

	return (int16_t)can_raw;
}

int16_t M3508::pid_calc(void)
{
	switch (m_Mode)
	{
		case 0: // Sleep
			return 0;

		case 1: // SpeedMode PID calculation
			return SpeedModeCalculation();

		case 2: // PosSpeedMode PID calculation
			return PosSpeedModeCalculation();

		case 3: // MIT mode (impedance / torque)
			return MITModeCalculation();

		default:
			break;
	}
	return 0;
}

/**
  * @brief  Append the calculated control value to the corresponding CAN Tx buffer
  * @param  Calcu_result: Calculated control value for M3508 motor
  * @retval return 1 if append success, else return 0
  */
uint8_t M3508::MsgAppend(int16_t Calcu_result)
{
	uint8_t Offset = ((m_ID - 1) % 4) * 2;// Read C620 data sheet
	uint8_t bus_idx = m_CAN->m_bus_idx;

	if (m_ID >= 1 && m_ID <= 4){
		TxGroup1[bus_idx].Data[Offset]     = (Calcu_result >> 8) & 0xFF;
		TxGroup1[bus_idx].Data[Offset + 1] = Calcu_result & 0xFF;
		return 1;
	}
	else if (m_ID >= 5 && m_ID <= 8){
		TxGroup2[bus_idx].Data[Offset]     = (Calcu_result >> 8) & 0xFF;
		TxGroup2[bus_idx].Data[Offset + 1] = Calcu_result & 0xFF;
		return 1;
	}
	return 0;
}

void M3508::SetZeroPoint(void)
{
	m_encoder_offset = -1;
}

/**
  * @brief  Sends M3508 motor CAN frames and clears the data buffer after transmission
  * @param  can: CANDevice instance
  * @param  identifier: CAN identifier (0x200, 0x1FF)
  * @retval 0: Invalid parameter / transmission failure
  *         1: Transmission success
  */
uint8_t M3508::SendGroup(CANDevice* can, uint16_t identifier)
{
	if (can == NULL){
		return 0;
	}

	uint8_t bus_idx = can->m_bus_idx;
	uint8_t SendState = 0;

	if (identifier == 0x200){
		SendState = can->SendMsg(&TxGroup1[bus_idx], 5);
		for(uint8_t i = 0; i < 8; i++){
			TxGroup1[bus_idx].Data[i] = 0;
		}
	}
	else if (identifier == 0x1FF){
		SendState = can->SendMsg(&TxGroup2[bus_idx], 5);
		for(uint8_t i = 0; i < 8; i++){
			TxGroup2[bus_idx].Data[i] = 0;
		}
	}
	else{
		return 0;
	}
	return SendState;
}

/**
 * @brief  Assigns CAN message to the corresponding M3508 motor object.
 * @param  can: CANDevice that received the message (reads can->m_RxMsg internally).
 * @return Pointer to the matched M3508 instance, or NULL if not registered/invalid.
 */
M3508* M3508::MsgAssign(CANDevice* can)
{
	if (can == NULL) return NULL;

	uint8_t bus_idx = can->m_bus_idx;
	if (bus_idx >= CANDevice::MAX_INSTANCES) return NULL;

	CanMsg* RxMsg = &can->m_RxMsg;
	if (RxMsg->ID >= 0x201 && RxMsg->ID <= 0x208)
	{
		uint16_t idx = RxMsg->ID - 0x201;
		if (M3508::MotorRegester[bus_idx][idx] != NULL)
		{
			if (M3508::MotorRegester[bus_idx][idx] -> ParseFeedback(RxMsg))
			{
				return M3508::MotorRegester[bus_idx][idx];
			}
		}
	}
	return NULL;
}

/**
 * @brief  Update motor control loop when a CAN message arrives.
 * @param  can: CANDevice that received the message (reads can->m_RxMsg internally).
 * @return 1 if motor updated, 0 if ID mismatch or data error.
 */
uint8_t M3508::ControlLoopUpdate(CANDevice* can)
{
	M3508* this_motor = MsgAssign(can);
	if (this_motor != NULL)
	{
		this_motor -> MsgAppend(this_motor -> pid_calc());
		return 1;
	}
	return 0;
}
