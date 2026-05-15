/*

  Identifier: 0x200 or 0x1FF																							  (canX)
	---------------------			  |---->  M2006::MsgAppend(int16_t Calcu_result)									  can bus											
	|	M2006_(1)result	|             |                                                                                      |-----	M2006_(1)	
	|			 		|		Be Appended				                                                                     |			 		
	|	M2006_(2)result	|	 according its offset		X = 1 or 2				                                             |-----	M2006_(2)	
	|			 		|	  ---------------->		CanMsg Motor_GroupX	---------->	M2006::SendGroups(&hcanX, identifier)----|
	|	M2006_(3)result	|							  	 		                             	(main.cpp)		             |----- M2006_(3)	
	|			 		|                                                                                                    |			 		
	|	M2006_(4)result	|                                                                                                    |-----	M2006_(4)	
	---------------------                                                                                                	 |
																															 | 
	M2006::Motor_Regester  -----> (address of instance M2006)																 |
		|----------|																										 |
   [0]	| m_ID = 1 | <--|																							   		 | Read Message
   [1]	| m_ID = 2 | <--|																									 |  (CanX_Msg)
   ...	|	 ...   | <--|----- M2006::Motor_Regester[idx] <-------- idx = ID - 0x201 <-------- ID <-------- message <--------|		 |
   [6]	| m_ID = 7 | <--|																 (0x201 ~ 0x208)							 |
   [7]	| m_ID = 8 | <--|																					   |					 V
		|----------|																						   |		  externed on can_drv.h
				|______________________________________________________________________________________________|
															  |
															  |
											 M2006* M2006::MsgAssign(CanMsg* msg)
*/
#include "M2006_Lib.h"

#define	k	10.0f / 10000.0f

// This group is used for identifier 0x200
static CanMsg M2006_Group1 = {0x200, CAN_ID_STD, CAN_RTR_DATA, 8, {0}};
// This group is used for identifier 0x1FF
static CanMsg M2006_Group2 = {0x1FF, CAN_ID_STD, CAN_RTR_DATA, 8, {0}};

// Regester of motor
M2006* M2006::Motor_Regester[8] = {NULL};

M2006::M2006(CAN_HandleTypeDef* hcan, uint8_t ID)
    : m_cascade_frq(3),
	  m_hcan(hcan),
      m_ID(ID),          
	  m_redRatio(36.0f),
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
      m_speed_pid(0.65, 0.01, 0.3, 1500.0f, 9000.0f),
      m_pos_pid(15, 0, 5, 1000.0f, 2000.0f)
{
	if (m_ID >= 1 && m_ID <= 8) 
	{
        Motor_Regester[m_ID - 1] = this;// Save address of motor according to its m_ID
    }
}

/**
  * @brief  Initialize parameters of position PID and speed PID
  * @param  Pos_Loop: Position loop PID parameters
  * @param  Speed_Loop: Speed loop PID parameters
  * @retval None
  */
void M2006::PID_Config(const PID& Pos_Loop, const PID& Speed_Loop)
{
	m_pos_pid = Pos_Loop;
	m_pos_pid = Speed_Loop;
}

void M2006::FeedForward_Config(float flexibility, float smoothness)
{
	m_speed_pid.SetFeedForward(flexibility, smoothness);
}


uint8_t M2006::ParseFeedback(CanMsg *RxMsg)
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
	
	int16_t Raw_Curr = (RxMsg->Data[4] << 8) | RxMsg->Data[5];
	m_Torque_Curr = k * Raw_Curr;									// -10 ~ 10
	
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

void M2006::SpeedMode(int16_t target_speed)
{
	m_speed_pid.setTarget(target_speed);
	m_Mode = 1; // 1 is SpeedMode
}


int16_t M2006::SpeedModeCalculation(void)
{
	m_speed_pid.setMeasure(m_Vel);
	
	int16_t result = (int16_t)m_speed_pid.calculate();//Do the PID calculation
	
	//Prevent overflow
	if (result >= 10000){
		result = 10000;
	}else if(result <= -10000){
		result = -10000;
	}
	
	return result;
}


void M2006::PosSpeedMode(float target_Angel, uint16_t Speed_Limit)
{	
	float target_counts = (target_Angel * m_redRatio * 8192.0f) / 360.0f;
	m_pos_pid.setTarget(target_counts);	// Set angle target(degree, output shaft)
	m_pos_pid.m_output_limit = Speed_Limit;	// Set Speed limitation
	m_Mode = 2; // Set mode as PosSpeed mode
}


int16_t M2006::PosSpeedModeCalculation(void)
{
	float pos_measure = (float)m_abs_Pos;
	
	if(m_cascade_frq == 3)
	{
		m_cascade_frq = 0;
		m_pos_pid.setMeasure(pos_measure);
		m_speed_pid.setTarget(m_pos_pid.calculate());
	}
	
	m_cascade_frq ++;
	
	m_speed_pid.setMeasure(m_Vel);
	return m_speed_pid.calculate();
}


int16_t M2006::pid_calc(void)
{
	switch (m_Mode)
	{
		case 0: // Sleep
			return 0;
		
		case 1: // SpeedMode PID calculation
			return SpeedModeCalculation();
	
		case 2: // PosSpeedMode PID calculation
			return PosSpeedModeCalculation();
		
		default:
			break;
	}
	return 0;
}

/**
  * @brief  Append the calculated control value to the corresponding CAN Tx buffer
  * @param  Calcu_result: Calculated control value for M2006 motor
  * @retval return 1 if append success, else return 0
  */
uint8_t M2006::MsgAppend(int16_t Calcu_result)
{
	uint8_t Offset = ((m_ID - 1) % 4) * 2;// Read C620 data sheet
	if (m_ID > 8){
		return 0;
	}
	
	if (m_ID <= 4){
		M2006_Group1.Data[Offset] 		= (Calcu_result >> 8) & 0xFF;
		M2006_Group1.Data[Offset + 1] 	= Calcu_result & 0xFF;
		return 1;
	}
	else{
		M2006_Group2.Data[Offset] 		= (Calcu_result >> 8) & 0xFF;
		M2006_Group2.Data[Offset + 1] 	= Calcu_result & 0xFF;
		return 1;
	}
}

void M2006::SetZeroPoint(void)
{
	m_encoder_offset = -1;
}

/**
  * @brief  Sends M2006 motor CAN frames and clears the data buffer after transmission
  * @param  hcan: CAN handle
  * @param  identifier: CAN identifier (0x200, 0x1FF)
  * @retval 0: Invalid parameter / transmission failure
  *         1: Transmission success
  */
uint8_t M2006::SendGroup(CAN_HandleTypeDef* hcan, uint16_t identifier)
{
	if (hcan == NULL){
		return 0;
	}
	
	uint8_t SendState = 0;
	
	// If send group with 0x200 identifier
	if (identifier == 0x200){
		SendState = CAN_Send_Msg(hcan, &M2006_Group1, 5);
		// Clear the data
		for(uint8_t i = 0; i <8; i++){
			M2006_Group1.Data[i] = 0;
		}
	}
	// If send group with 0x1FF identifier
	else if (identifier == 0x1FF){
		SendState = CAN_Send_Msg(hcan, &M2006_Group2, 5);
		// Clear the data
		for(uint8_t i = 0; i <8; i++){
			M2006_Group2.Data[i] = 0;
		}
	}
	else{
		return 0;
	}
	return SendState;
}

/**
 * @brief  Assigns CAN message to the corresponding M2006 motor object.
 * @param  RxMsg: Pointer to the received CAN message structure.
 * @return Pointer to the matched M2006 motor instance, or NULL if not registered/invalid.
 */
M2006* M2006::MsgAssign(CanMsg* RxMsg)
{
	if (RxMsg->ID >= 0x201 && RxMsg->ID <= 0x208)
	{
		uint16_t idx = RxMsg->ID - 0x201;
		if (M2006::Motor_Regester[idx] != NULL) // Check whether this position is registed
		{
			if (M2006::Motor_Regester[idx] -> ParseFeedback(RxMsg)) // OMG!!!
			{
				return M2006::Motor_Regester[idx]; // return only when parsing success
			}
		}
	}
	return NULL;
}

/**
 * @brief  Update motor control loop when a CAN message arrives.
 * @param  RxMsg: The raw CAN data from the bus.
 * @return 1 if motor updated, 0 if ID mismatch or data error.
 */
uint8_t M2006::ControlLoopUpdate(CanMsg* RxMsg)
{
	M2006* this_motor = M2006::MsgAssign(RxMsg); // Who get this message
	if (this_motor != NULL) // Check whether it is valid
	{
		this_motor -> MsgAppend(this_motor -> pid_calc()); // Do PID calculation
		return 1;
	}
	return 0;
}
