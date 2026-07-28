/**
  ******************************************************************************
  * @file    MPU6050.h
  * @brief   STM32 MPU6050 Driver (C++ Version) — Integrated Mahony AHRS
  ******************************************************************************
  */

/*
 * File Name        : MPU6050.h
 * Description      : C++ MPU6050 driver with built-in Mahony attitude estimation
 *                    - 6-axis IMU (3-axis Gyro + 3-axis Accel) + Temperature
 *                    - I2C register read/write via I2CDevice
 *                    - On-chip Mahony AHRS → Euler angles
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : i2c_drv.h, Mahony.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-27
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

/* ========================== MPU6050 Register Map ========================== */
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

/* ======================== Clock Source Options ======================== */
#define     MPU6050_CLK_INTERNAL       0x00
#define     MPU6050_CLK_GYRO_X         0x01
#define     MPU6050_CLK_GYRO_Y         0x02
#define     MPU6050_CLK_GYRO_Z         0x03
#define     MPU6050_CLK_EXT_32K        0x04
#define     MPU6050_CLK_EXT_19M        0x05
#define     MPU6050_CLK_STOP           0x07

/* ======================== Gyroscope Range Options ======================== */
#define     MPU6050_GYRO_250DEG        0x00
#define     MPU6050_GYRO_500DEG        0x01
#define     MPU6050_GYRO_1000DEG       0x02
#define     MPU6050_GYRO_2000DEG       0x03

/* ======================== Accelerometer Range Options ======================== */
#define     MPU6050_ACCEL_2G           0x00
#define     MPU6050_ACCEL_4G           0x01
#define     MPU6050_ACCEL_8G           0x02
#define     MPU6050_ACCEL_16G          0x03

/* ======================== DLPF Bandwidth Options ======================== */
#define     MPU6050_DLPF_260HZ         0x00
#define     MPU6050_DLPF_184HZ         0x01
#define     MPU6050_DLPF_94HZ          0x02
#define     MPU6050_DLPF_44HZ          0x03
#define     MPU6050_DLPF_21HZ          0x04
#define     MPU6050_DLPF_10HZ          0x05
#define     MPU6050_DLPF_5HZ           0x06

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Mahony.h"

/**
 * @brief  MPU6050 Configuration Structure
 * @note   Pass to MPU6050 constructor; all hardware init happens inside
 */
struct MPU6050Config {
	uint8_t DLPF;               // DLPF bandwidth (MPU6050_DLPF_xxx)
	uint8_t GyroRange;          // Gyroscope full-scale (MPU6050_GYRO_xxx)
	uint8_t AccelRange;         // Accelerometer full-scale (MPU6050_ACCEL_xxx)
	uint8_t SampleRateDiv;      // Sample rate divider (0 ~ 255)
	float   Kp;                 // Mahony proportional gain
	float   Ki;                 // Mahony integral gain
};

/**
 * @brief  MPU6050 Sensor Class with integrated Mahony AHRS
 */
class MPU6050 {

public:
	/* Raw sensor readings (LSB) */
	int16_t m_RawAccX, m_RawAccY, m_RawAccZ;
	int16_t m_RawGyroX, m_RawGyroY, m_RawGyroZ;
	int16_t m_Temp;

	/* Euler angles */
	float m_Roll, m_Pitch, m_Yaw;
	float m_GyroX, m_GyroY, m_GyroZ;
	float m_AccX, m_AccY, m_AccZ;

	/* On-chip Mahony attitude estimator */
	Mahony m_Mahony;

private:
	I2CDevice      *m_i2c;          // Pointer to I2C bus device
	MPU6050Config   m_Config;       // Hardware configuration
	float           m_OffsetGyroX, m_OffsetGyroY, m_OffsetGyroZ;  // Gyro zero-rate offset (LSB)
	float           m_GyroScale;    // Raw LSB → rad/s  conversion factor
	float           m_AccelScale;   // Raw LSB → g      conversion factor

public:
	/**
	 * @brief  Constructor — configures MPU6050 registers and initializes Mahony
	 * @param  i2c:    pointer to I2CDevice (e.g. &i2c2)
	 * @param  config: MPU6050 hardware & Mahony parameter configuration
	 */
	MPU6050(I2CDevice *i2c, const MPU6050Config &config);

	/**
	 * @brief  Write a single byte to an MPU6050 register
	 */
	uint8_t WriteReg(uint8_t RegAddress, uint8_t Data);

	/**
	 * @brief  Read a single byte from an MPU6050 register
	 */
	uint8_t ReadReg(uint8_t RegAddress);

	/**
	 * @brief  Read all sensor data, run Mahony AHRS, update Euler angles(degree)
	 * @note   Call at fixed intervals (configured by SampleFrq)
	 *         Updates: m_AccX/Y/Z, m_GyroX/Y/Z, m_Temp, m_Roll, m_Pitch, m_Yaw
	 */
	uint8_t GetData();

	/**
	 * @brief  Gyroscope zero-rate offset calibration (IMU must be stationary)
	 * @note   Reads 1000 samples, averages raw LSB, stores in m_OffsetGyroX/Y/Z
	 * @retval 1 on success, 0 if too many I2C failures
	 */
	uint8_t CalibrateGyro();
};

#endif /* __cplusplus */

#endif /* __MPU6050_H */
