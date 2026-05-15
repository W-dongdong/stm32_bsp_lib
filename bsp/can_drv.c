/**
  ******************************************************************************
  * @file    can_drv.c
  * @brief   STM32 CAN Driver Wrapper
  ******************************************************************************
  */

/*
 * File Name        : can_drv.c
 * Description      : HAL library based CAN driver wrapper for RoboMaster
 *                    - 1 ID per filter bank, only check ID (ignores frame type)
 *                    - CAN interrupt activation function
 *                    - Standard frame (11-bit ID) transmission
 *                    - Standard/Extended frame reception
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-03-29
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "can_drv.h"//change the head file when necessary

/**
 * @brief  Set CAN filter
 * @param  hcan: CAN handle pointer (&hcan1, &hcan2)
 * @param  FilterBank: Filter bank number (CAN1:0-13, CAN2:14-27)
 * @param  FilterFIFO: CAN_RX_FIFO0 or CAN_RX_FIFO1
 * @param  ID: 11-bit standard ID (0x000-0x7FF)
 * @retval HAL status
 * @note   One ID matches one filter, only check ID, ignores frame type.
 * @note   You must enable CAN bus after using this function
 */
HAL_StatusTypeDef CAN_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t FilterBank, uint32_t FilterFIFO, uint16_t ID)
{
    CAN_FilterTypeDef sFilterConfig;

    // Parameter check
    assert_param(hcan != NULL);
    assert_param(FilterFIFO == CAN_RX_FIFO0 || FilterFIFO == CAN_RX_FIFO1);

    sFilterConfig.FilterBank = FilterBank;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = FilterFIFO;

    sFilterConfig.FilterIdHigh = (ID & 0x7FF) << 5;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	
	return HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
}

/**
  * @brief  Enable CAN interrupts.
  * @param  hcan: Pointer to CAN handle
  * @param  ActiveITs: CAN interrupts to enable (combination of @arg CAN_Interrupts)
  * @retval HAL status
  * @note   You must enable CAN bus after using this function and
  *			config CAN interrupt in cubeMX
  */
HAL_StatusTypeDef CAN_IT_Config(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
    return HAL_CAN_ActivateNotification(hcan, ActiveITs);
}

/**
  * @brief  Enable CAN bus.
  * @param  hcan: Pointer to CAN handle
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_Start(CAN_HandleTypeDef *hcan)
{
	return  HAL_CAN_Start(hcan);
}

/**
 * @brief  CAN Send Message function
 * @param  hcan: Pointer to HAL CAN handle
 * @param  TxMsg: Pointer to CanMsg struct
 * @param  time_out: Timeout in milliseconds for TX mailbox (The maximum time you expect to spend on this function)
 * @retval 1: Message successfully queued to TX mailbox; 0: Failed
 * @note   ONLY supports STANDARD ID (CAN_ID_STD) and DATA FRAME (CAN_RTR_DATA)
 * @note   Standard ID range: 0x000 ~ 0x7FF; DLC range: 0 ~ 8
 * @IMPORTANT   DO NOT FORGET MODIFY YOUR DLC OF TxMsg!!!
 */
uint8_t CAN_Send_Msg(CAN_HandleTypeDef* hcan, CanMsg *TxMsg, uint32_t time_out)
{
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint32_t mailbox;
    HAL_StatusTypeDef status;

    // Parameter validity check
	if ((hcan == NULL) || (TxMsg == NULL) || (TxMsg->DLC > 8) || (TxMsg->ID > 0x7FF))
    {
        return 0;
    }

    // Configure HAL TX header
	if (TxMsg->IDE == CAN_ID_STD)
	{
		CAN_TxHeader.StdId = TxMsg->ID;
		CAN_TxHeader.ExtId = 0x00000000;
	}
	else if(TxMsg -> IDE == CAN_ID_EXT)
	{
		CAN_TxHeader.StdId = 0x0000;
		CAN_TxHeader.ExtId = TxMsg->ID;
	}
    CAN_TxHeader.IDE = TxMsg -> IDE;
    CAN_TxHeader.RTR = TxMsg -> RTR;
    CAN_TxHeader.DLC = TxMsg->DLC;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
	
	uint32_t tick_start = HAL_GetTick();
    while (1)
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TME0) ||
            __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TME1) ||
            __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TME2))
        {
            break;
        }

        if (HAL_GetTick() - tick_start > time_out)
        {
            return 0;
        }
	}

    // Queue message to free TX mailbox
    status = HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, TxMsg->Data, &mailbox);
    
    // Return 1 only if queuing
    return (status == HAL_OK) ? 1 : 0;
}

/**
 * @brief Read CAN message via HAL library, store into custom struct
 * @param hcan: Pointer to CAN handle
 * @param RxFIFO: CAN RX FIFO number (CAN_RX_FIFO0 / CAN_RX_FIFO1)
 * @param Rx_Msg: Pointer to struct to store received ID, DLC, data
 * @retval 1: Success, 0: Fail
 */
uint8_t CAN_Read_Msg(CAN_HandleTypeDef *hcan, uint32_t RxFIFO, CanMsg *RxMsg)
{
    CAN_RxHeaderTypeDef RxHeader;
    HAL_StatusTypeDef Status;
	
	// Parameter validity check
	if ((hcan == NULL) || (RxMsg == NULL) || (RxMsg->DLC > 8) || ((RxFIFO != CAN_RX_FIFO0)&& (RxFIFO != CAN_RX_FIFO1)))
    {
        return 0;
    }
	

    Status = HAL_CAN_GetRxMessage(hcan, RxFIFO, &RxHeader, RxMsg -> Data);//Recieve data
    if(Status != HAL_OK)
    {
        return 0;
    }

    RxMsg -> DLC = (uint8_t)RxHeader.DLC;
	RxMsg -> IDE = (uint8_t)RxHeader.IDE;
	RxMsg -> RTR = (uint8_t)RxHeader.RTR;

    // Assign ID (STD/EXT)
    if(RxHeader.IDE == CAN_ID_STD)
    {
        RxMsg -> ID = RxHeader.StdId;
    }
    else if(RxHeader.IDE == CAN_ID_EXT)
    {
        RxMsg -> ID = RxHeader.ExtId;
    }
	else
	{
		return 0;
	}
    return 1;
}


CanMsg Can1_Msg;// save message from can1
CanMsg Can2_Msg;// save message from can1
uint8_t CAN1_RxFlag = 0;// can1 recieve flag
uint8_t CAN2_RxFlag = 0;// can2 recieve flag

/*CAN FIFO0 interrupt recieve function*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan -> Instance == CAN1)
	{
		CAN_Read_Msg(hcan, CAN_RX_FIFO0, &Can1_Msg);
		CAN1_RxFlag = 1;
	}
	else
	{
		CAN_Read_Msg(hcan, CAN_RX_FIFO0, &Can2_Msg);
		CAN2_RxFlag = 1;
	}
}

/*CAN FIFO1 interrupt recieve function*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan -> Instance == CAN1)
	{
		CAN_Read_Msg(hcan, CAN_RX_FIFO1, &Can1_Msg);
		CAN1_RxFlag = 1;
	}
	else
	{
		CAN_Read_Msg(hcan, CAN_RX_FIFO1, &Can2_Msg);
		CAN2_RxFlag = 1;
	}
}
/*存在Fifo覆盖的风险，可以直接在这里面就执行M3508Assign，拿到就分发就不会覆盖了*/
