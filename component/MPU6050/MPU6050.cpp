/**
  ******************************************************************************
  * @file    MPU6050.cpp
  * @brief   STM32 MPU6050 Driver (C++ Version) — Implementation
  ******************************************************************************
  */

/*
 * File Name        : MPU6050.cpp
 * Description      : C++ MPU6050 driver with built-in Mahony attitude estimation
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : MPU6050.h
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-27
 *
 * Team Notes:
 * ATTENTION: Before you modify the code, make sure that you understand your code and modified function
 */

#include "MPU6050.h"

#define DEG2RAD  (3.1415926535898f / 180.0f)

// ============================================================================
// Constructor — initializes MPU6050 registers + Mahony AHRS
// ============================================================================

MPU6050::MPU6050(I2CDevice *i2c, const MPU6050Config &config)
	: m_AccX(0), m_AccY(0), m_AccZ(0)
	, m_GyroX(0), m_GyroY(0), m_GyroZ(0)
	, m_Temp(0)
	, m_Roll(0.0f), m_Pitch(0.0f), m_Yaw(0.0f)
	, m_AHRS(config.Kp, config.Ki, config.SampleFrq)
	, m_i2c(i2c)
	, m_Config(config)
	, m_GyroScale(0.0f)
	, m_AccelScale(0.0f)
{
	/* ---- Compute scale factors: LSB → rad/s (gyro), LSB → g (accel) ---- */
	/* Full-scale range: gyro = 250 << GyroRange, accel = 2 << AccelRange */
	/* Conversion: range / 32768.0f (16-bit signed ADC) */
	m_GyroScale  = (float)(250 << config.GyroRange)  / 32768.0f * DEG2RAD;
	m_AccelScale = (float)(2   << config.AccelRange) / 32768.0f;

	/* ---- Hardware initialization ---- */
	WriteReg(MPU6050_PWR_MGMT_1,  0x80);                      // Device reset
	// HAL_Delay(100);  ← uncomment if your platform needs reset settling time

	WriteReg(MPU6050_PWR_MGMT_1,  config.ClockSource);        // Clock source & wake up
	WriteReg(MPU6050_SMPLRT_DIV,  config.SampleRateDiv);      // Sample rate divider
	WriteReg(MPU6050_CONFIG,      config.DLPF);               // DLPF bandwidth
	WriteReg(MPU6050_GYRO_CONFIG, config.GyroRange  << 3);    // Gyroscope range
	WriteReg(MPU6050_ACCEL_CONFIG,config.AccelRange << 3);    // Accelerometer range
}

// ============================================================================
// Write a single byte to an MPU6050 register
// ============================================================================

uint8_t MPU6050::WriteReg(uint8_t RegAddress, uint8_t Data)
{
	if (m_i2c == NULL) {
		return 0;
	}
	return m_i2c->MemWrite(MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10);
}

// ============================================================================
// Read a single byte from an MPU6050 register
// ============================================================================

uint8_t MPU6050::ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	if (m_i2c == NULL) {
		return 0;
	}
	if (m_i2c->MemRead(MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10)) {
		return Data;
	}
	return 0;
}

// ============================================================================
// GetData — read sensors → Mahony AHRS → Euler angles
// ============================================================================

void MPU6050::GetData()
{
	if (m_i2c == NULL) {
		return;
	}

	/* ---- Read 14 bytes starting from ACCEL_XOUT_H ---- */
	uint8_t RawData[14];
	if (m_i2c->MemRead(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, RawData, 14, 10))
	{
		/* Parse big-endian raw data into member variables */
		m_AccX  = (int16_t)((RawData[0]  << 8) | RawData[1]);
		m_AccY  = (int16_t)((RawData[2]  << 8) | RawData[3]);
		m_AccZ  = (int16_t)((RawData[4]  << 8) | RawData[5]);
		m_Temp  = (int16_t)((RawData[6]  << 8) | RawData[7]);
		m_GyroX = (int16_t)((RawData[8]  << 8) | RawData[9]);
		m_GyroY = (int16_t)((RawData[10] << 8) | RawData[11]);
		m_GyroZ = (int16_t)((RawData[12] << 8) | RawData[13]);
	}

	/* ---- Convert raw LSB → physical units ---- */
	float ax = (float)m_AccX  * m_AccelScale;   // g
	float ay = (float)m_AccY  * m_AccelScale;
	float az = (float)m_AccZ  * m_AccelScale;
	float gx = (float)m_GyroX * m_GyroScale;    // rad/s
	float gy = (float)m_GyroY * m_GyroScale;
	float gz = (float)m_GyroZ * m_GyroScale;

	/* ---- Mahony AHRS quaternion update ---- */
	m_AHRS.SixAxisUpdate(gx, gy, gz, ax, ay, az);

	/* ---- Extract Euler angles from quaternion ---- */
	m_AHRS.GetEulerAngle(&m_Pitch, &m_Yaw, &m_Roll);
}
