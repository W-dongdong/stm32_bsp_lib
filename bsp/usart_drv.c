#include "usart_drv.h"


/**
 * @brief  Send 8-bit data
 * @param  huart: Pointer to UART handle
 * @param  byte: 8-bit data to send
 * @param  timeout: Timeout in milliseconds
 * @retval 1: Success, 0: Failure/Timeout
 */
uint8_t UART_Send_U8(UART_HandleTypeDef *huart, uint8_t byte, uint32_t timeout)
{
    if (huart == NULL || timeout == 0){
        return 0;
	}
    return (HAL_UART_Transmit(huart, &byte, 1, timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Send 16-bit data seperately
 * @param  huart: Pointer to UART handle
 * @param  U16_Data: 16-bit data to send
 * @param  timeout: Timeout in milliseconds
 * @retval 1: Success, 0: Failure/Timeout
 * @note   Little-endian by default
 */
uint8_t UART_Send_U16(UART_HandleTypeDef *huart, uint16_t U16_Data, uint32_t timeout)
{
    if (huart == NULL || timeout == 0){
        return 0;
	}
    return (HAL_UART_Transmit(huart, (uint8_t*)&U16_Data, 2, timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Send 32-bit data seperately
 * @param  huart: Pointer to UART handle
 * @param  U32_Data: 32-bit data to send
 * @param  timeout: Timeout in milliseconds
 * @retval 1: Success, 0: Failure/Timeout
 * @note   Little-endian by default
 */
uint8_t UART_Send_U32(UART_HandleTypeDef *huart, uint32_t U32_Data, uint32_t timeout)
{
    if (huart == NULL || timeout == 0){
        return 0;
	}
    return (HAL_UART_Transmit(huart, (uint8_t*)&U32_Data, 4, timeout) == HAL_OK) ? 1 : 0;
}

uint8_t UART_Read_U8(UART_HandleTypeDef *huart, uint8_t *p_Data, uint32_t timeout)
{
	if (huart == NULL || p_Data == NULL || timeout == 0){
		return 0;
	}
	return HAL_UART_Receive(huart, p_Data, 1, timeout) == HAL_OK ? 1 : 0;
}

uint8_t UART_Send_Block(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len, uint32_t timeout) 
{
    if (huart == NULL || pData == NULL || len == 0) return 0;
    return (HAL_UART_Transmit(huart, pData, len, timeout) == HAL_OK) ? 1 : 0;
}

/*现在可以到stm32f4xx_it.c这个文件里面找一下串口和DMA的函数，写这个函数
串口DMA停止，串口DMA开始，串口DMA接收，串口DMA接收长度计算*/

uint8_t UART_DMA_Stop(UART_HandleTypeDef *huart)
{
	if (huart == NULL){
		return 0;
	}
	return (HAL_UART_DMAStop(huart) == HAL_OK) ? 1 : 0;
}

uint8_t UART_DMA_Start(UART_HandleTypeDef *huart, uint8_t *usart_rx_buf, uint16_t USART_RX_BUF_LEN)
{
	if (huart == NULL){
		return 0;
	}
	return (HAL_UART_Receive_DMA(huart, usart_rx_buf, USART_RX_BUF_LEN)  == HAL_OK) ? 1: 0;
}

uint8_t UART_DMA_Recieve(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	return (HAL_UART_Receive_DMA(huart, pData, Size) == HAL_OK) ? 1 : 0;
}

// These function seems like serving for DMA and IT configuration
/*---------------------------------------------------*/
///* --Here are the Error recovery function--- */
//void UART_Overrun_Handle(UART_HandleTypeDef *huart)
//{
//	UART_printf(huart, "Overrun Error");
//	/* Start of your error handler */
//	
//	/* End of your error handler */
//}
//void UART_FrameError_Handle(UART_HandleTypeDef *huart)
//{
//	UART_printf(huart, "Frame Error");
//	/* Start of your error handler */
//	
//	/* End of your error handler */
//}
//void UART_NoiseError_Handle(UART_HandleTypeDef *huart)
//{
//	UART_printf(huart, "Noise Error");
//	/* Start of your error handler */
//	
//	/* End of your error handler */
//}
//void UART_DMAError_Handle(UART_HandleTypeDef *huart)
//{
//	UART_printf(huart, "DMA Error");
//	/* Start of your error handler */
//	  HAL_UART_DMAStop(huart);
//    
//    // Check whether DMA is configuration
//    if (huart->hdmarx != NULL) {
//        HAL_UART_Receive_DMA(huart, rx_buf, RX_LEN);
//    }
//	/* End of your error handler */
//}

//void UART_ErrorHandler(UART_HandleTypeDef *huart)
//{
//	uint32_t error = huart->ErrorCode;

//    if (error & HAL_UART_ERROR_ORE) {
//        UART_Overrun_Handle(huart);
//    }
//    if (error & HAL_UART_ERROR_FE) {
//        UART_FrameError_Handle(huart);
//    }
//    if (error & HAL_UART_ERROR_NE){
//        UART_NoiseError_Handle(huart);
//    }
//	if (error & HAL_UART_ERROR_DMA) {
//    UART_DMAError_Handle(huart);
//	}
//}


///* ---Here are the callback function--- */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//	UART_ErrorHandler(huart);
//}
/*---------------------------------------------------*/

/*通用性有待考究*/
/*自动化开启串口空闲中断和DMA接收*/
/*使用前需要在cubeMX里面开启串口全局中断*/
uint8_t UART_IdleRx_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if ( (huart == NULL) || (pData == NULL)){
		return 0;
	}
	return (HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size) == HAL_OK) ? 1 : 0;
}

/*
THis function is used for USART IDLE interrupt and DMA with normal mode
Both USART IDLE interrup and DMA full interrup will jump to this function
WHen you use it you should rewrite it after the main function
*/
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	
//}


/*UART printf function*/
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
  return ch;
}

/**
 * @brief  High-efficiency multi-UART printf (Blocking mode)
 * @param  huart: Pointer to UART handle
 * @param  format: Printf-style format string
 * @param  ...: Optional arguments
 * @retval 1: Success, 0: Failure/Timeout
 * @note   This function cut off the overflow string
 */
uint8_t UART_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    static char buf[128]; // Prevent stack overflow
    va_list args;
    va_start(args, format);

    // Plug '\0' at the end of buf
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len > 0)
    {
        // Whether the length of expected message longer than 128
		// Maximum length is 127 and 1 byte for '\0' at the end
        uint16_t send_len = (len < (int)sizeof(buf)) ? (uint16_t)len : (uint16_t)(sizeof(buf) - 1);

        return UART_Send_Block(huart, (uint8_t *)buf, send_len, 15);
    }
	return 0;
}

/**
 * @brief  High-efficiency UART DMA printf (Non-blocking)
 * @param  huart: Pointer to UART handle
 * @param  format: Printf-style format string
 * @param  ...: Optional arguments
 * @retval 1: Success, 0: Failure/Busy/DMA-Not-Configured
 * @note   This function cut off the overflow string
 */
uint8_t UART_DMA_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    static char buf[128]; 

    // Check whether DMA is enable
    if (huart->hdmatx == NULL)
    {
        return 0;
    }

    // Check whether DMA is busy
    if (huart->gState != HAL_UART_STATE_READY) 
    {
        return 0; // Abandon the data
    }

    va_list args;
    va_start(args, format);

    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len > 0)
    {
        // Whether the length of expected message longer than 128
		// Maximum length is 127 and 1 byte for '\0' at the end
        uint16_t send_len = (len < (int)sizeof(buf)) ? (uint16_t)len : (uint16_t)(sizeof(buf) - 1);

		// Whether DMA is started
        if (HAL_UART_Transmit_DMA(huart, (uint8_t *)buf, send_len) == HAL_OK)
        {
            return 1;
        }
    }
    return 0;
}

/*
串口DMA函数，串口中断函数，包括接收中断，发送中断，定时器函数，PWM函数，GPIO点灯函数
*/
