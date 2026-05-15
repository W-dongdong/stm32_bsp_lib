#include "i2c_drv.h"


/**
 * @brief Checks if the target I2C device is online.
 * @param hi2c: Pointer to I2C handle
 * @param DevAddress: Target device address
 * @param Trials: Number of transmission attempts.
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    return (HAL_I2C_IsDeviceReady(hi2c, DevAddress, Trials, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Transmits raw data to a slave device (No register addressing).
 * @param hi2c: Pointer to I2C handle.
 * @param DevAddress: Target device address
 * @param pData: Pointer to data buffer to be sent.
 * @param Size: Amount of data to be sent (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_MasterTransmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Receives raw data from a slave device (No register addressing).
 * @param hi2c: Pointer to I2C handle.
 * @param DevAddress: Target device address
 * @param pData: Pointer to data buffer to store received bytes.
 * @param Size: Amount of data to be received (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_MasterReceive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Writes data to a specific device register.
 * @param hi2c: Pointer to I2C handle.
 * @param DevAddress: Target device address (Left-shifted).
 * @param MemAddress: Internal register address to write to.
 * @param MemAddSize: Size of internal address. Use:
 *                    [ I2C_MEMADD_SIZE_8BIT ] or [ I2C_MEMADD_SIZE_16BIT ]
 * @param pData: Pointer to data buffer to be written.
 * @param Size: Amount of data to be written (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_MemWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Reads data from a specific device register.
 * @param hi2c: Pointer to I2C handle.
 * @param DevAddress: Target device address (Left-shifted).
 * @param MemAddress: Internal register address to read from.
 * @param MemAddSize: Size of internal address. Use:
 *                    [ I2C_MEMADD_SIZE_8BIT ] or [ I2C_MEMADD_SIZE_16BIT ]
 * @param pData: Pointer to data buffer to store read bytes.
 * @param Size: Amount of data to be read (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_MemRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Transmits data to a master when acting as a slave.
 * @param hi2c: Pointer to I2C handle.
 * @param pData: Pointer to data buffer to be sent.
 * @param Size: Amount of data to be sent (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_SlaveTransmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Slave_Transmit(hi2c, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

/**
 * @brief Receives data from a master when acting as a slave.
 * @param hi2c: Pointer to I2C handle.
 * @param pData: Pointer to data buffer to store received bytes.
 * @param Size: Amount of data to be received (in bytes).
 * @param Timeout: Timeout duration in ms.
 * @retval 1: Success, 0: Error/Timeout.
 */
uint8_t I2C_SlaveReceive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return (HAL_I2C_Slave_Receive(hi2c, pData, Size, Timeout) == HAL_OK) ? 1 : 0;
}

