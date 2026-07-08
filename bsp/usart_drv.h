/**
  ******************************************************************************
  * @file    usart_drv.h
  * @brief   STM32 USART Driver Wrapper (C++ Version) - Header
  ******************************************************************************
  */

/*
 * File Name        : usart_drv.h
 * Description      : C++ USART driver wrapper class
 *                    - UART Send (U8/U16/U32/Block)
 *                    - UART DMA + IDLE line receive
 *                    - UART printf (blocking & DMA)
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : usart.h (HAL UART peripheral header)
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __USART_DRV_H
#define __USART_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief  USART Device Class
 * @note   Create instance with &huart1, &huart2, etc. to select UART peripheral.
 *         Instances are registered in creation order.
 *         Example: USARTDevice usart1(&huart1);
 */
class USARTDevice {
public:

    UART_HandleTypeDef *m_huart;
    uint8_t   m_RxFlag;
    uint16_t  m_RxLen;
    uint8_t   m_TxBuf[128];

    static const uint8_t MAX_INSTANCES = 6;
    static USARTDevice *instances_[MAX_INSTANCES];
    static uint8_t    instance_count_;

    USARTDevice(UART_HandleTypeDef *huart);
    uint8_t Send_U8(uint8_t byte, uint32_t timeout);
    uint8_t Send_U16(uint16_t U16_Data, uint32_t timeout);
    uint8_t Send_U32(uint32_t U32_Data, uint32_t timeout);
    uint8_t Send_Block(uint8_t *pData, uint16_t len, uint32_t timeout);
    uint8_t Read_U8(uint8_t *pData, uint32_t timeout);
    uint8_t DMA_Start(uint8_t *usart_rx_buf, uint16_t USART_RX_BUF_LEN);
    uint8_t DMA_Stop();
    uint8_t IdleRx_DMA(uint8_t *pData, uint16_t Size);
    uint8_t printf(const char *format, ...);
    uint8_t DMA_printf(const char *format, ...);
};

#endif /* __cplusplus */

#endif /* __USART_DRV_H */
