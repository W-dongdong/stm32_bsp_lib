/**
  ******************************************************************************
  * @file    can_drv.h
  * @brief   STM32 CAN Driver Wrapper Header File
  ******************************************************************************
  */

/*
 * File Name        : can_drv.h
 * Description      : Header file for HAL library based CAN driver wrapper
 * 
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can.h (HAL CAN peripheral header)
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-03-29
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __CAN_DRV_H
#define __CAN_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/*Include*/
#include "can.h"

/*User difine*/

/**
 * @brief  CAN receive message structure definition
 */
typedef struct{
    uint32_t ID;       // CAN ID (Standard or Extended)
    uint8_t  IDE;      // ID type (CAN_ID_STD / CAN_ID_EXT)
    uint8_t  RTR;      // Frame type (Data frame / Remote frame)
    uint8_t  DLC;      // Data length code (0~8)
    uint8_t  Data[8];  // CAN data buffer (Fixed 8 bytes)
} CanMsg;

/*Variables define*/
extern uint8_t CAN1_RxFlag;
extern uint8_t CAN2_RxFlag;
extern CanMsg Can1_Msg;
extern CanMsg Can2_Msg;

/*Function*/
HAL_StatusTypeDef CAN_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t FilterBank, uint32_t FilterFIFO, uint16_t ID);
HAL_StatusTypeDef CAN_IT_Config(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef CAN_Start(CAN_HandleTypeDef *hcan);
uint8_t CAN_Read_Msg(CAN_HandleTypeDef *hcan, uint32_t RxFIFO, CanMsg *Rx_Msg);
uint8_t CAN_Send_Msg(CAN_HandleTypeDef* hcan, CanMsg *TxMsg, uint32_t time_out);

#ifdef __cplusplus
}
#endif

#endif /*__CAN_DRV_H*/
