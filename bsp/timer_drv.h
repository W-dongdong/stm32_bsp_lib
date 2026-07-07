/**
  ******************************************************************************
  * @file    timer_drv.h
  * @brief   STM32 Timer Driver Wrapper (C++ Version) - Header
  ******************************************************************************
  */

/*
 * File Name        : timer_drv.h
 * Description      : C++ Timer driver wrapper class for RoboMaster
 *                    - Basic timer (period elapsed interrupt only)
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : tim.h (HAL TIM peripheral header)
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __TIMER_DRV_H
#define __TIMER_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "tim.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief  Timer Device Class
 * @note   Create instance with &htim1, &htim2, etc. to select timer peripheral.
 *         Instances are registered in creation order.
 *         Example: TIMDevice tim1(&htim1);
 */
class TIMDevice {
public:

    TIM_HandleTypeDef *m_htim;
    uint8_t   m_Flag;

    static TIMDevice *instances_[MAX_INSTANCES];
    static uint8_t    instance_count_;
    static constexpr uint8_t MAX_INSTANCES = 14;

    TIMDevice(TIM_HandleTypeDef *htim);
    uint8_t Start();
};

#endif /* __cplusplus */

#endif /* __TIMER_DRV_H */
