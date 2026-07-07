/**
  ******************************************************************************
  * @file    MPU6050.h
  * @brief   STM32 MPU6050 Driver (C++ Version) - Header
  ******************************************************************************
  */

/*
 * File Name        : MPU6050.h
 * Description      : C++ MPU6050 driver class
 *                    - 6-axis IMU (3-axis Gyro + 3-axis Accel)
 *                    - I2C register read/write via I2CDevice
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : i2c_drv.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-07
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "i2c_drv.h"

/* MPU6050 Register Map */
#define     MPU6050_SMPLRT_DIV        0x19
#define     MPU6050_CONFIG             0x1A
#define     MPU6050_GYRO_CONFIG        0x1B
#define     MPU6050_ACCEL_CONFIG       0x1C

#define     MPU6050_ACCEL_XOUT_H       0x3B
#define     MPU6050_ACCEL_XOUT_L       0x3C
#define     MPU6050_ACCEL_YOUT_H       0x3D
#define     MPU6050_ACCEL_YOUT_L       0x3E
#define     MPU6050_ACCEL_ZOUT_H       0x3F
#define     MPU6050_ACCEL_ZOUT_L       0x40
#define     MPU6050_TEMP_OUT_H         0x41
#define     MPU6050_TEMP_OUT_L         0x42
#define     MPU6050_GYRO_XOUT_H        0x43
#define     MPU6050_GYRO_XOUT_L        0x44
#define     MPU6050_GYRO_YOUT_H        0x45
#define     MPU6050_GYRO_YOUT_L        0x46
#define     MPU6050_GYRO_ZOUT_H        0x47
#define     MPU6050_GYRO_ZOUT_L        0x48

#define     MPU6050_PWR_MGMT_1         0x6B
#define     MPU6050_PWR_MGMT_2         0x6C
#define     MPU6050_WHO_AM_I           0x75

#define     MPU6050_ADDRESS            0xD0

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief  MPU6050 Sensor Class
 * @note   Create with I2CDevice pointer to select I2C bus.
 *         Example:
 *           I2CDevice i2c2(&hi2c2);
 *           MPU6050   imu(&i2c2);
 *           imu.Init();
 *           imu.GetData(&AccX, &AccY, &AccZ, &Temp, &GyroX, &GyroY, &GyroZ);
 */
class MPU6050 {

private:
    I2CDevice *m_i2c;

public:
    MPU6050(I2CDevice *i2c);

    uint8_t WriteReg(uint8_t RegAddress, uint8_t Data);
    uint8_t ReadReg(uint8_t RegAddress);
    void Init();
    void GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *Temp,
                 int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
};

#endif /* __cplusplus */

#endif /* __MPU6050_H */
