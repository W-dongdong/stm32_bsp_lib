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
	: m_RawAccX(0), m_RawAccY(0), m_RawAccZ(0)
	, m_RawGyroX(0), m_RawGyroY(0), m_RawGyroZ(0)
	, m_Temp(0)
	, m_Roll(0.0f), m_Pitch(0.0f), m_Yaw(0.0f)
	, m_AccX(0), m_AccY(0), m_AccZ(0)
	, m_GyroX(0), m_GyroY(0), m_GyroZ(0)
	, m_Mahony((config.DLPF != MPU6050_DLPF_260HZ)
	           ? 1000.0f / (1 + config.SampleRateDiv)
	           : 8000.0f,
	           config.Kp, config.Ki)
	, m_i2c(i2c)
	, m_Config(config)
	, m_OffsetGyroX(0), m_OffsetGyroY(0), m_OffsetGyroZ(0)
	, m_GyroScale(0.0f)
	, m_AccelScale(0.0f)
{
	/* ---- Compute scale factors: LSB -> rad/s (gyro), LSB -> g (accel) ---- */
	m_GyroScale  = (float)(250 << config.GyroRange)  / 32768.0f * DEG2RAD;
	m_AccelScale = (float)(2   << config.AccelRange) / 32768.0f;

	/* ---- Hardware initialization ---- */
	WriteReg(MPU6050_PWR_MGMT_1,  MPU6050_CLK_GYRO_X);        // Clock source & wake up
	WriteReg(MPU6050_PWR_MGMT_2,  0x00);                      // All axes active
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
// CalibrateGyro — 1000-sample gyroscope zero-rate offset calibration
// ============================================================================

/* Attention: Please make sure that the IMU is static before calling this function */
uint8_t MPU6050::CalibrateGyro()
{
	if (m_i2c == NULL) {
		return 0;
	}

	int32_t sumGX = 0, sumGY = 0, sumGZ = 0;
	uint16_t valid = 0;
	uint8_t RawData[6];

	for (uint16_t i = 0; i < 1000; i++) {
		if (m_i2c->MemRead(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H,
		                   I2C_MEMADD_SIZE_8BIT, RawData, 6, 10)) {
			sumGX += (int16_t)((RawData[0] << 8) | RawData[1]);
			sumGY += (int16_t)((RawData[2] << 8) | RawData[3]);
			sumGZ += (int16_t)((RawData[4] << 8) | RawData[5]);
			valid++;
		}
	}

	if (valid < 500) {
		return 0;
	}

	float avgGX = (float)sumGX / valid;
	float avgGY = (float)sumGY / valid;
	float avgGZ = (float)sumGZ / valid;

	m_OffsetGyroX = avgGX;
	m_OffsetGyroY = avgGY;
	m_OffsetGyroZ = avgGZ;
	return 1;
}

// ============================================================================
// GetData — read sensors -> Mahony AHRS -> Euler angles
// ============================================================================

uint8_t MPU6050::GetData()
{
	if (m_i2c == NULL) {
		return 0;
	}

	/* ---- Read 14 bytes starting from ACCEL_XOUT_H ---- */
	uint8_t RawData[14];
	if (m_i2c->MemRead(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, RawData, 14, 10))
	{
		/* Parse big-endian raw data into member variables */
		m_RawAccX  = (int16_t)((RawData[0]  << 8) | RawData[1]);
		m_RawAccY  = (int16_t)((RawData[2]  << 8) | RawData[3]);
		m_RawAccZ  = (int16_t)((RawData[4]  << 8) | RawData[5]);
		m_Temp  = (int16_t)((RawData[6]  << 8) | RawData[7]);
		m_RawGyroX = (int16_t)((RawData[8]  << 8) | RawData[9]);
		m_RawGyroY = (int16_t)((RawData[10] << 8) | RawData[11]);
		m_RawGyroZ = (int16_t)((RawData[12] << 8) | RawData[13]);

		/* ---- Convert raw LSB -> physical units ---- */
		m_AccX = (float)m_RawAccX   * m_AccelScale;   // g
		m_AccY = (float)m_RawAccY   * m_AccelScale;
		m_AccZ = (float)m_RawAccZ   * m_AccelScale;
		m_GyroX = ((float)m_RawGyroX - m_OffsetGyroX) * m_GyroScale;    // rad/s
		m_GyroY = ((float)m_RawGyroY - m_OffsetGyroY) * m_GyroScale;
		m_GyroZ = ((float)m_RawGyroZ - m_OffsetGyroZ) * m_GyroScale;

		/* ---- Mahony AHRS quaternion update ---- */
		m_Mahony.SixAxisUpdate(m_GyroX, m_GyroY, m_GyroZ, m_AccX, m_AccY, m_AccZ);

		/* ---- Extract Euler angles from quaternion ---- */
		if (m_Mahony.GetEulerAngle(&m_Pitch, &m_Yaw, &m_Roll)) {
			return 1;
		}
	}
	return 0;
}
