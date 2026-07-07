/**
  ******************************************************************************
  * @file    i2c_drv.cpp
  * @brief   STM32 I2C Driver Wrapper (C++ Version) - Implementation
  ******************************************************************************
  */

/*
 * File Name        : i2c_drv.cpp
 * Description      : C++ I2C driver wrapper class
 *                    - Device ready check
 *                    - Master transmit / receive (raw)
 *                    - Memory write / read (register-based)
 *                    - Slave transmit / receive
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : i2c_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "i2c_drv.h"

I2CDevice *I2CDevice::instances_[I2CDevice::MAX_INSTANCES] = {NULL};
uint8_t    I2CDevice::instance_count_ = 0;

I2CDevice::I2CDevice(I2C_HandleTypeDef *hi2c)
    : m_hi2c(hi2c)
    , m_RxFlag(0)
{
    if (m_hi2c == NULL){
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
 * @brief Checks if the target I2C device is online.
 * @param DevAddress: Target device address
 * @param Trials: Number of transmission attempts.
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::IsDeviceReady(uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    return (HAL_I2C_IsDeviceReady(m_hi2c, DevAddress, Trials, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Transmits raw data to a slave device (No register addressing).
 * @param DevAddress: Target device address
 * @param pData: Pointer to data buffer to be sent.
 * @param Size: Amount of data to be sent (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::MasterTransmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Master_Transmit(m_hi2c, DevAddress, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Receives raw data from a slave device (No register addressing).
 * @param DevAddress: Target device address
 * @param pData: Pointer to data buffer to store received bytes.
 * @param Size: Amount of data to be received (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::MasterReceive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Master_Receive(m_hi2c, DevAddress, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Writes data to a specific device register.
 * @param DevAddress: Target device address (Left-shifted).
 * @param MemAddress: Internal register address to write to.
 * @param MemAddSize: Size of internal address. Use:
 *                    [ I2C_MEMADD_SIZE_8BIT ] or [ I2C_MEMADD_SIZE_16BIT ]
 * @param pData: Pointer to data buffer to be written.
 * @param Size: Amount of data to be written (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::MemWrite(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Mem_Write(m_hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Reads data from a specific device register.
 * @param DevAddress: Target device address (Left-shifted).
 * @param MemAddress: Internal register address to read from.
 * @param MemAddSize: Size of internal address. Use:
 *                    [ I2C_MEMADD_SIZE_8BIT ] or [ I2C_MEMADD_SIZE_16BIT ]
 * @param pData: Pointer to data buffer to store read bytes.
 * @param Size: Amount of data to be read (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::MemRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Mem_Read(m_hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Transmits data to a master when acting as a slave.
 * @param pData: Pointer to data buffer to be sent.
 * @param Size: Amount of data to be sent (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::SlaveTransmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Slave_Transmit(m_hi2c, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Receives data from a master when acting as a slave.
 * @param pData: Pointer to data buffer to store received bytes.
 * @param Size: Amount of data to be received (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success
 * @retval 0: Error/Timeout
 */
uint8_t I2CDevice::SlaveReceive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Slave_Receive(m_hi2c, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

// ============================================================================
// HAL I2C Interrupt Callbacks (extern "C")
// ============================================================================

extern "C" {

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < I2CDevice::MAX_INSTANCES; i++){
        if ((I2CDevice::instances_[i] != NULL) &&
            (I2CDevice::instances_[i]->m_hi2c == hi2c)){
            I2CDevice::instances_[i]->m_RxFlag = 1;
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < I2CDevice::MAX_INSTANCES; i++){
        if ((I2CDevice::instances_[i] != NULL) &&
            (I2CDevice::instances_[i]->m_hi2c == hi2c)){
            I2CDevice::instances_[i]->m_RxFlag = 1;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < I2CDevice::MAX_INSTANCES; i++){
        if ((I2CDevice::instances_[i] != NULL) &&
            (I2CDevice::instances_[i]->m_hi2c == hi2c)){
            /* Error handling -- user to implement */
        }
    }
}

} // extern "C"
