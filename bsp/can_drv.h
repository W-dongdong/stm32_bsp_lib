/**
  ******************************************************************************
  * @file    can_drv.h
  * @brief   STM32 CAN Driver Wrapper (C++ Version) - Header
  ******************************************************************************
  */

/*
 * File Name        : can_drv.h
 * Description      : C++ CAN driver wrapper class
 *                    - 1 ID per filter bank, only check ID (ignores frame type)
 *                    - CAN interrupt activation function
 *                    - Standard frame (11-bit ID) transmission
 *                    - Standard/Extended frame reception
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can.h (HAL CAN peripheral header)
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __CAN_DRV_H
#define __CAN_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "can.h"

/**
 * @brief  CAN receive message structure definition
 */
typedef struct {
    uint32_t ID;       // CAN ID (Standard or Extended)
    uint8_t  IDE;      // ID type (CAN_ID_STD / CAN_ID_EXT)
    uint8_t  RTR;      // Frame type (Data frame / Remote frame)
    uint8_t  DLC;      // Data length code (0~8)
    uint8_t  Data[8];  // CAN data buffer (Fixed 8 bytes)
} CanMsg;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief  CAN Device Class
 * @note   Create instance with &hcan1 or &hcan2 to select CAN bus.
 *         Instances are registered in creation order.
 *         Example: CANDevice can1(&hcan1);
 */
class CANDevice {
public:

    CanMsg    m_RxMsg;
    uint8_t   m_RxFlag;
    uint8_t   m_bus_idx;
    CAN_HandleTypeDef *m_hcan;

    static const uint8_t MAX_INSTANCES = 3;
    static CANDevice *instances_[MAX_INSTANCES];
    static uint8_t    instance_count_;

    CANDevice(CAN_HandleTypeDef *hcan);
    uint8_t Filter_Config(uint8_t FilterBank, uint32_t FilterFIFO, uint16_t ID);
    uint8_t IT_Config(uint32_t ActiveITs);
    uint8_t Start();
    uint8_t Send_Msg(CanMsg *TxMsg, uint32_t time_out);
    uint8_t Read_Msg(uint32_t RxFIFO, CanMsg *RxMsg);
};

#endif /* __cplusplus */

#endif /* __CAN_DRV_H */
