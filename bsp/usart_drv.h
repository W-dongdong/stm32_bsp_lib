#ifndef __USART_DRV_H
#define __USART_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/*Include*/
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
	
/*Function declaration*/
uint8_t UART_Send_U8(UART_HandleTypeDef *huart, uint8_t byte, uint32_t timeout);
uint8_t UART_Send_U16(UART_HandleTypeDef *huart, uint16_t U16_Data, uint32_t timeout);
uint8_t UART_Send_U32(UART_HandleTypeDef *huart, uint32_t U32_Data, uint32_t timeout);
uint8_t UART_Send_Block(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len, uint32_t timeout);
	
uint8_t UART_DMA_Stop(UART_HandleTypeDef *huart);
uint8_t UART_DMA_Start(UART_HandleTypeDef *huart, uint8_t *usart_rx_buf, uint16_t USART_RX_BUF_LEN);
	
uint8_t UART_printf(UART_HandleTypeDef *huart, const char *format, ...);	
uint8_t UART_DMA_printf(UART_HandleTypeDef *huart, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /*__USART_DRV_H*/
