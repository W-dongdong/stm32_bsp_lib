/**
  ******************************************************************************
  * @file    DM3519_Lib.h
  * @brief   DM3519 Motor Driver Library - Header
  ******************************************************************************
  */

/*
 * File Name        : DM3519_Lib.h
 * Description      : DM3519 motor driver library using CANDevice abstraction.
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __DM3519_LIB_H
#define __DM3519_LIB_H

#include "can_drv.h"

typedef enum
{
    disable = 0,
    enable  = 1
} MotorState;

class DM3519{
private:
    CANDevice* m_CAN;

public:

    uint16_t   m_ID;          // Motor ID
    float     m_redRatio;     // Reduction Ratio (default 3591/187)

    // ========== Feedback ==========
    uint8_t  m_Error;         // Error flag
    uint16_t m_Pos;           // Encoder position
    int16_t  m_Vel;           // Velocity
    uint16_t m_Torque;        // Torque current
    uint8_t  m_T_MOS;         // Average MOS temperature
    uint8_t  m_T_Rotor;       // Rotor temperature

    // ========== Target ==========
    float    m_SetSpeed;      // Target speed of output shaft (rad/s)
    float    m_SetPos;        // Target position of output shaft (degree)

    DM3519(CANDevice* can, uint16_t ID);

    // ========== Feedback ==========
    uint8_t ParseFeedback(CanMsg* RxMsg);

    // ========== Control ==========
    uint8_t SetMotorState(MotorState State);
    uint8_t SpeedMode(float target_rads);
    uint8_t PosSpeedMode(float Pos, float Speed);
};

#endif
