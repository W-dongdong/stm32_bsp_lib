#ifndef __I2C_DRV_H
#define __I2C_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/*Include*/
#include "i2c.h"

/*User difine*/


/*Variables define*/


/*Function*/
uint8_t I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
uint8_t I2C_MasterTransmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t I2C_MasterReceive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t I2C_MemWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t I2C_MemRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t I2C_SlaveTransmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t I2C_SlaveReceive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);


#ifdef __cplusplus
}
#endif

#endif /*__I2C_DRV_H*/
