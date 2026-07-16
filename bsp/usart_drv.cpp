/**
  ******************************************************************************
  * @file    usart_drv.cpp
  * @brief   STM32 USART Driver Wrapper (C++ Version) - Implementation
  ******************************************************************************
  */

/*
 * File Name        : usart_drv.cpp
 * Description      : C++ USART driver wrapper class
 *                    - UART Send (U8/U16/U32/Block)
 *                    - UART DMA + IDLE line receive
 *                    - UART printf (blocking & DMA)
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : usart_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "usart_drv.h"

USARTDevice *USARTDevice::instances_[USARTDevice::MAX_INSTANCES] = {NULL};
uint8_t    USARTDevice::instance_count_ = 0;

USARTDevice::USARTDevice(UART_HandleTypeDef *huart)
    : m_huart(huart)
    , m_RxFlag(0)
    , m_RxLen(0)
{
    if (m_huart == NULL){
        return;
    }

    if (instance_count_ < MAX_INSTANCES){
        instances_[instance_count_++] = this;
    }
}

// ============================================================================
// Public Methods
// ============================================================================

/**
 * @brief  Send 8-bit data
 * @param  byte: 8-bit data to send
 * @param  timeout: Timeout in milliseconds
 * @retval 1: Success
 * @retval 0: Failure/Timeout
 */
uint8_t USARTDevice::Send_U8(uint8_t byte, uint32_t timeout)
{
    if (m_huart == NULL || timeout == 0){
        return 0;
    }
    return (HAL_UART_Transmit(m_huart, &byte, 1, timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Send 16-bit data seperately
 * @param  U16_Data: 16-bit data to send
 * @param  timeout: Timeout in milliseconds
 * @retval 1: Success
 * @retval 0: Failure/Timeout
 * @note   Little-endian by default
 */
uint8_t USARTDevice::Send_U16(uint16_t U16_Data, uint32_t timeout)
{
    if (m_huart == NULL || timeout == 0){
        return 0;
    }
    return (HAL_UART_Transmit(m_huart, (uint8_t*)&U16_Data, 2, timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Send 32-bit data seperately
 * @param  U32_Data: 32-bit data to send
 * @param  timeout: Timeout in milliseconds
 * @retval 1: Success
 * @retval 0: Failure/Timeout
 * @note   Little-endian by default
 */
uint8_t USARTDevice::Send_U32(uint32_t U32_Data, uint32_t timeout)
{
    if (m_huart == NULL || timeout == 0){
        return 0;
    }
    return (HAL_UART_Transmit(m_huart, (uint8_t*)&U32_Data, 4, timeout) == HAL_OK) ? 1 : 0;
}

uint8_t USARTDevice::Send_Block(uint8_t *pData, uint16_t len, uint32_t timeout)
{
    if (m_huart == NULL || pData == NULL || len == 0) return 0;
    return (HAL_UART_Transmit(m_huart, pData, len, timeout) == HAL_OK) ? 1 : 0;
}

uint8_t USARTDevice::Read_U8(uint8_t *pData, uint32_t timeout)
{
    if (m_huart == NULL || pData == NULL || timeout == 0){
        return 0;
    }
    return (HAL_UART_Receive(m_huart, pData, 1, timeout) == HAL_OK) ? 1 : 0;
}

uint8_t USARTDevice::DMA_Stop()
{
    if (m_huart == NULL){
        return 0;
    }
    return (HAL_UART_DMAStop(m_huart) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Start non-blocking UART DMA reception.
 * @param usart_rx_buf Buffer to store incoming data.
 * @param USART_RX_BUF_LEN Number of bytes to receive.
 * @return 1 on success, 0 on failure.
 * @note This function returns immediately. The HAL callback 
 *       HAL_UART_RxCpltCallback will be triggered ONLY after the DMA 
 *       has fully received the specified length of bytes.
 */
uint8_t USARTDevice::DMA_Start(uint8_t *usart_rx_buf, uint16_t USART_RX_BUF_LEN)
{
    if (m_huart == NULL){
        return 0;
    }
    return (HAL_UART_Receive_DMA(m_huart, usart_rx_buf, USART_RX_BUF_LEN) == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Start non-blocking UART TX using DMA.
 * @param  pTxBuffer Pointer to the data buffer to be transmitted.
 * @param  send_len  Number of bytes to send.
 * @return 1 if transmission started successfully, 0 otherwise.
 * @note   Returns immediately. HAL_UART_TxCpltCallback is triggered 
 *         ONLY after the DMA has finished sending the specified bytes.
 */
uint8_t USARTDevice::Tx_DMA(uint8_t* pTxBuffer, uint16_t send_len)
{
    if (m_huart == NULL) return 0;
    if (HAL_UART_Transmit_DMA(m_huart, pTxBuffer, send_len) == HAL_OK)
    {
        return 1;
    }
    return 0;
}

uint8_t USARTDevice::IdleRx_DMA(uint8_t *pData, uint16_t Size)
{
    if ((m_huart == NULL) || (pData == NULL)){
        return 0;
    }
    return (HAL_UARTEx_ReceiveToIdle_DMA(m_huart, pData, Size) == HAL_OK) ? 1 : 0;
}

/* UART printf function */
/**
 * @brief  High-efficiency multi-UART printf (Blocking mode)
 * @param  format: Printf-style format string
 * @param  ...: Optional arguments
 * @retval 1: Success
 * @retval 0: Failure/Timeout
 * @note   This function cut off the overflow string
 */
uint8_t USARTDevice::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // Plug '\0' at the end of buf
    int len = vsnprintf((char *)m_TxBuf, sizeof(m_TxBuf), format, args);
    va_end(args);

    if (len > 0)
    {
        // Whether the length of expected message longer than 128
        // Maximum length is 127 and 1 byte for '\0' at the end
        uint16_t send_len = (len < (int)sizeof(m_TxBuf)) ? (uint16_t)len : (uint16_t)(sizeof(m_TxBuf) - 1);

        return Send_Block(m_TxBuf, send_len, 15);
    }
    return 0;
}

/**
 * @brief  High-efficiency UART DMA printf (Non-blocking)
 * @param  format: Printf-style format string
 * @param  ...: Optional arguments
 * @retval 1: Success
 * @retval 0: Failure/Busy/DMA-Not-Configured
 * @note   This function cut off the overflow string
 */
uint8_t USARTDevice::DMA_printf(const char *format, ...)
{
    // Check whether DMA is enable
    if (m_huart->hdmatx == NULL)
    {
        return 0;
    }

    // Check whether DMA is busy
    if (m_huart->gState != HAL_UART_STATE_READY)
    {
        return 0; // Abandon the data
    }

    va_list args;
    va_start(args, format);

    int len = vsnprintf((char *)m_TxBuf, sizeof(m_TxBuf), format, args);
    va_end(args);

    if (len > 0)
    {
        // Whether the length of expected message longer than 128
        // Maximum length is 127 and 1 byte for '\0' at the end
        uint16_t send_len = (len < (int)sizeof(m_TxBuf)) ? (uint16_t)len : (uint16_t)(sizeof(m_TxBuf) - 1);

        // Whether DMA is started
        if (HAL_UART_Transmit_DMA(m_huart, m_TxBuf, send_len) == HAL_OK)
        {
            return 1;
        }
    }
    return 0;
}

// ============================================================================
// HAL UART Interrupt Callbacks (extern "C")
// ============================================================================

extern "C" {

/*
This function is used for USART IDLE interrupt and DMA with normal mode
Both USART IDLE interrupt and DMA full interrupt will jump to this function
When you use it you should rewrite it after the main function
*/

/**
 * @brief  UART Rx Event callback (IDLE / DMA transfer complete)
 * @note   Iterates registered USARTDevice instances, dispatches to matched huart.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < USARTDevice::MAX_INSTANCES; i++){
        if ((USARTDevice::instances_[i] != NULL) &&
            (USARTDevice::instances_[i]->m_huart == huart)){
            USARTDevice::instances_[i]->m_RxLen = Size;
            USARTDevice::instances_[i]->m_RxFlag = 1;
        }
    }
}

/**
 * @brief  UART Error callback
 * @note   Dispatches to matched USARTDevice instance.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < USARTDevice::MAX_INSTANCES; i++){
        if ((USARTDevice::instances_[i] != NULL) &&
            (USARTDevice::instances_[i]->m_huart == huart)){
            /* Error handling -- user to implement */
        }
    }
}

} // extern "C"
