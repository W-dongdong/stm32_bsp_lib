/**
  ******************************************************************************
  * @file    can_drv.cpp
  * @brief   STM32 CAN Driver Wrapper (C++ Version) - Implementation
  ******************************************************************************
  */

/*
 * File Name        : can_drv.cpp
 * Description      : C++ CAN driver wrapper class
 *                    - 1 ID per filter bank, only check ID (ignores frame type)
 *                    - CAN interrupt activation function
 *                    - Standard frame (11-bit ID) transmission
 *                    - Standard/Extended frame reception
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : can_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "can_drv.h"

CANDevice *CANDevice::instances_[CANDevice::MAX_INSTANCES] = {NULL};
uint8_t    CANDevice::instance_count_ = 0;

CANDevice::CANDevice(CAN_HandleTypeDef *hcan)
    : m_RxFlag(0)
    , m_bus_idx(0)
    , m_hcan(hcan)
{
	if (m_hcan == NULL){
		return;
	}

	m_RxMsg.ID   = 0;
	m_RxMsg.IDE  = 0;
	m_RxMsg.RTR  = 0;
	m_RxMsg.DLC  = 0;
	for (uint8_t i = 0; i < 8; i++){
		m_RxMsg.Data[i] = 0;
	}

	if (instance_count_ < MAX_INSTANCES){
		m_bus_idx = instance_count_;
		instances_[instance_count_++] = this;
	}
	
	HAL_CAN_Start(m_hcan);
}

// ============================================================================
// Public Methods
// ============================================================================

/**
  * @brief  Configure CAN filter (1 ID per filter bank)
  * @param  FilterBank: Filter bank number (CAN1: 0-13, CAN2: 14-27)
  * @param  FilterFIFO: CAN_RX_FIFO0 or CAN_RX_FIFO1
  * @param  ID: 11-bit standard ID (0x000--0x7FF)
  * @retval 1: Success
  * @retval 0: Failure (bad parameter or HAL error)
  */
uint8_t CANDevice::Filter_Config(uint8_t  FilterBank, uint32_t FilterFIFO, uint16_t ID)
{
	CAN_FilterTypeDef sFilterConfig;

	if ((m_hcan == NULL) ||
	    ((FilterFIFO != CAN_RX_FIFO0) && (FilterFIFO != CAN_RX_FIFO1))){
		return 0;
	}

	sFilterConfig.FilterBank           = FilterBank;
	sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = FilterFIFO;
	sFilterConfig.FilterIdHigh         = (ID & 0x7FF) << 5;
	sFilterConfig.FilterIdLow          = 0x0000;
	sFilterConfig.FilterMaskIdHigh     = 0x7FF << 5;
	sFilterConfig.FilterMaskIdLow      = 0x0000;
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;

	return (HAL_CAN_ConfigFilter(m_hcan, &sFilterConfig) == HAL_OK) ? 1 : 0;
}

/**
  * @brief  Enable CAN interrupts
  * @param  ActiveITs: CAN interrupt sources to enable
  * @retval 1: Success
  * @retval 0: Failure (null handle or HAL error)
  */
uint8_t CANDevice::IT_Config(uint32_t ActiveITs)
{
	if (m_hcan == NULL){
		return 0;
	}
	return (HAL_CAN_ActivateNotification(m_hcan, ActiveITs) == HAL_OK) ? 1 : 0;
}

/**
  * @brief  Start CAN peripheral
  * @retval 1: Success
  * @retval 0: Failure (null handle or HAL error)
  */
uint8_t CANDevice::Start()
{
	if (m_hcan == NULL){
		return 0;
	}
	return (HAL_CAN_Start(m_hcan) == HAL_OK) ? 1 : 0;
}

/**
  * @brief  Transmit CAN message (Standard or Extended ID)
  * @param  TxMsg: Pointer to CanMsg (IDE decides STD/EXT routing)
  * @param  time_out: TX mailbox timeout in milliseconds
  * @retval 1: Message queued to TX mailbox
  * @retval 0: Failure (bad parameter, timeout, or HAL error)
  * @note   STD ID: 0x000--0x7FF (11-bit);  EXT ID: 0x000--0x1FFFFFFF (29-bit)
  * @note   IMPORTANT: Set TxMsg->DLC before calling
  */
uint8_t CANDevice::SendMsg(CanMsg *TxMsg, uint32_t time_out)
{
	CAN_TxHeaderTypeDef CAN_TxHeader;
	uint32_t            mailbox;
	uint32_t            tick_start;

	if ((m_hcan == NULL) || (TxMsg == NULL) || (TxMsg->DLC > 8)){
		return 0;
	}

	if (TxMsg->IDE == CAN_ID_STD){
		if (TxMsg->ID > 0x7FF){
			return 0;
		}
	} else if (TxMsg->IDE == CAN_ID_EXT){
		if (TxMsg->ID > 0x1FFFFFFF){
			return 0;
		}
	} else {
		return 0;
	}

	if (TxMsg->IDE == CAN_ID_STD){
		CAN_TxHeader.StdId = TxMsg->ID;
		CAN_TxHeader.ExtId = 0x00000000;
	} else {
		CAN_TxHeader.StdId = 0x000;
		CAN_TxHeader.ExtId = TxMsg->ID;
	}
	CAN_TxHeader.IDE                = TxMsg->IDE;
	CAN_TxHeader.RTR                = TxMsg->RTR;
	CAN_TxHeader.DLC                = TxMsg->DLC;
	CAN_TxHeader.TransmitGlobalTime = DISABLE;

	tick_start = HAL_GetTick();
	for (;;){
		if (__HAL_CAN_GET_FLAG(m_hcan, CAN_FLAG_TME0) ||
		    __HAL_CAN_GET_FLAG(m_hcan, CAN_FLAG_TME1) ||
		    __HAL_CAN_GET_FLAG(m_hcan, CAN_FLAG_TME2)){
			break;
		}
		if ((HAL_GetTick() - tick_start) > time_out){
			return 0;   /* Timeout -- no free mailbox */
		}
	}

	return (HAL_CAN_AddTxMessage(m_hcan, &CAN_TxHeader, TxMsg->Data, &mailbox) == HAL_OK) ? 1 : 0;
}

/**
  * @brief  Read received CAN message from FIFO
  * @param  RxFIFO: CAN_RX_FIFO0 or CAN_RX_FIFO1
  * @param  RxMsg: [out] Pointer to CanMsg to populate
  * @retval 1: Message read successfully
  * @retval 0: Failure (null pointer, bad FIFO, no message, or HAL error)
  * @note   RxMsg is an OUTPUT parameter -- pre-call content is ignored
  */
uint8_t CANDevice::ReadMsg(uint32_t RxFIFO, CanMsg *RxMsg)
{
	CAN_RxHeaderTypeDef RxHeader;

	if ((m_hcan == NULL) || (RxMsg == NULL)){
		return 0;
	}
	if ((RxFIFO != CAN_RX_FIFO0) && (RxFIFO != CAN_RX_FIFO1)){
		return 0;
	}

	if (HAL_CAN_GetRxMessage(m_hcan, RxFIFO, &RxHeader, RxMsg->Data) != HAL_OK){
		return 0;
	}

	RxMsg->DLC = (uint8_t)RxHeader.DLC;
	RxMsg->IDE = (uint8_t)RxHeader.IDE;
	RxMsg->RTR = (uint8_t)RxHeader.RTR;

	if (RxHeader.IDE == CAN_ID_STD){
		RxMsg->ID = RxHeader.StdId;
	} else if (RxHeader.IDE == CAN_ID_EXT){
		RxMsg->ID = RxHeader.ExtId;
	} else {
		return 0;   /* Unknown IDE -- exception guard */
	}

	return 1;
}

// ============================================================================
// HAL CAN Interrupt Callbacks (extern "C")
// ============================================================================

extern "C" {

/**
  * @brief  CAN RX FIFO0 message pending callback
  * @note   Iterates registered CANDevice instances, dispatches to matched hcan.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	for (uint8_t i = 0; i < CANDevice::MAX_INSTANCES; i++){
		if ((CANDevice::instances_[i] != NULL) &&
		    (CANDevice::instances_[i]->m_hcan == hcan)){
			CANDevice::instances_[i]->ReadMsg(CAN_RX_FIFO0,
				&(CANDevice::instances_[i]->m_RxMsg));
			CANDevice::instances_[i]->m_RxFlag = 1;
		}
	}
}

/**
  * @brief  CAN RX FIFO1 message pending callback
  * @note   Same dispatch logic as FIFO0, targeting FIFO1.
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	for (uint8_t i = 0; i < CANDevice::MAX_INSTANCES; i++){
		if ((CANDevice::instances_[i] != NULL) &&
		    (CANDevice::instances_[i]->m_hcan == hcan)){
			CANDevice::instances_[i]->ReadMsg(CAN_RX_FIFO1,
				&(CANDevice::instances_[i]->m_RxMsg));
			CANDevice::instances_[i]->m_RxFlag = 1;
		}
	}
}

} // extern "C"
