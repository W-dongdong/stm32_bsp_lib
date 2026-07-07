/**
  ******************************************************************************
  * @file    MPU6050.cpp
  * @brief   STM32 MPU6050 Driver (C++ Version) - Implementation
  ******************************************************************************
  */

/*
 * File Name        : MPU6050.cpp
 * Description      : C++ MPU6050 driver class
 *                    - 6-axis IMU (3-axis Gyro + 3-axis Accel)
 *                    - I2C register read/write via I2CDevice
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : MPU6050.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "MPU6050.h"

// ============================================================================
// Constructor
// ============================================================================

MPU6050::MPU6050(I2CDevice *i2c)
    : m_i2c(i2c)
{
    
}


uint8_t MPU6050::WriteReg(uint8_t RegAddress, uint8_t Data)
{
    if (m_i2c == NULL){
        return 0;
    }
    return m_i2c->MemWrite(MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10);
}

uint8_t MPU6050::ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    if (m_i2c == NULL){
        return 0;
    }
    if (m_i2c->MemRead(MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10))
    {
        return Data;
    }
    return 0;
}

void MPU6050::Init()
{
    WriteReg(MPU6050_PWR_MGMT_1,  0x01); // 解除休眠，选择陀螺仪时钟
    WriteReg(MPU6050_PWR_MGMT_1,  0x00); // 唤醒并设置时钟源
    WriteReg(MPU6050_SMPLRT_DIV,  0x09); // 采样率分频为10
    WriteReg(MPU6050_CONFIG,      0x06); // 滤波器带宽配置
    WriteReg(MPU6050_GYRO_CONFIG, 0x18); // 陀螺仪量程配置
    WriteReg(MPU6050_ACCEL_CONFIG,0x18); // 加速度计量程配置
}

void MPU6050::GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *Temp,
                       int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    if (m_i2c == NULL){
        return;
    }

    uint8_t RawData[14];
    if (m_i2c->MemRead(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, RawData, 14, 10))
    {
        *AccX  = (RawData[0]  << 8) | RawData[1];
        *AccY  = (RawData[2]  << 8) | RawData[3];
        *AccZ  = (RawData[4]  << 8) | RawData[5];
        *Temp  = (RawData[6]  << 8) | RawData[7];
        *GyroX = (RawData[8]  << 8) | RawData[9];
        *GyroY = (RawData[10] << 8) | RawData[11];
        *GyroZ = (RawData[12] << 8) | RawData[13];
    }
}
