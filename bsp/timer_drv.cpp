/**
  ******************************************************************************
  * @file    timer_drv.cpp
  * @brief   STM32 Timer Driver Wrapper (C++ Version) - Implementation
  ******************************************************************************
  */

/*
 * File Name        : timer_drv.cpp
 * Description      : C++ Timer driver wrapper class
 *                    - Basic timer (period elapsed interrupt only)
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : timer_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "timer_drv.h"

TIMDevice *TIMDevice::instances_[TIMDevice::MAX_INSTANCES] = {NULL};
uint8_t    TIMDevice::instance_count_ = 0;

TIMDevice::TIMDevice(TIM_HandleTypeDef *htim)
    : m_htim(htim)
    , m_Flag(0)
{
    if (m_htim == NULL){
        return;
    }

    if (instance_count_ < MAX_INSTANCES){
        instances_[instance_count_++] = this;
    }
}

// ============================================================================
// Public Methods
// ============================================================================

uint8_t TIMDevice::Start()
{
    if (m_htim == NULL){
        return 0;
    }
    return (HAL_TIM_Base_Start_IT(m_htim) == HAL_OK) ? 1 : 0;
}

// ============================================================================
// HAL Timer Interrupt Callbacks (extern "C")
// ============================================================================

extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < TIMDevice::MAX_INSTANCES; i++){
        if ((TIMDevice::instances_[i] != NULL) &&
            (TIMDevice::instances_[i]->m_htim == htim)){
            TIMDevice::instances_[i]->m_Flag = 1;
        }
    }
}

} // extern "C"
