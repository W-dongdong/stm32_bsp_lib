/**
  ******************************************************************************
  * @file    i2c_drv.h
  * @brief   STM32 I2C Driver Wrapper (C++ Version) - Header
  ******************************************************************************
  */

/*
 * File Name        : i2c_drv.h
 * Description      : C++ I2C driver wrapper
 *                    - Device ready check
 *                    - Master transmit / receive (raw)
 *                    - Memory write / read (register-based)
 *                    - Slave transmit / receive
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : i2c.h (HAL I2C peripheral header)
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __I2C_DRV_H
#define __I2C_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "i2c.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief  I2C Device Class
 * @note   Create instance with &hi2c1, &hi2c2, etc. to select I2C peripheral.
 *         Instances are registered in creation order.
 *         Example: I2CDevice i2c1(&hi2c1);
 */
class I2CDevice {
public:

    I2C_HandleTypeDef *m_hi2c;
    uint8_t   m_RxFlag;

    static I2CDevice *instances_[MAX_INSTANCES];
    static uint8_t    instance_count_;
    static constexpr uint8_t MAX_INSTANCES = 3;

    I2CDevice(I2C_HandleTypeDef *hi2c);
    uint8_t IsDeviceReady(uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
    uint8_t MasterTransmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    uint8_t MasterReceive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    uint8_t MemWrite(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    uint8_t MemRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    uint8_t SlaveTransmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
    uint8_t SlaveReceive(uint8_t *pData, uint16_t Size, uint32_t Timeout);
};

#endif /* __cplusplus */

#endif /* __I2C_DRV_H */
