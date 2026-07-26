/**
  ******************************************************************************
  * @file    Mahony.h
  * @brief   C++ Mahony AHRS Algorithm Class (6-Axis: Gyro + Accel, 9-Axis planned)
  ******************************************************************************
  */

/*
 * File Name        : Mahony.h
 * Description      : C++ Mahony AHRS filter with inline PI control
 *                    Flat structure — zero nesting, zero external class dependency
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : <cmath>
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-07-27
 *
 * Team Notes:
 * Before you modify the code, make sure that you understand your code and modified function
 *
 */

#ifndef __MAHONY_H
#define __MAHONY_H

#include <cmath>

class Mahony {
public:
	float m_q[4];              // Quaternion [w, x, y, z]
	float m_SampleFrq;         // IMU sample frequency in Hz
	float m_Kp;                // Proportional gain
	float m_Ki;                // Integral gain

private:
	float m_IntegralFBx;       // Integral feedback term X
	float m_IntegralFBy;       // Integral feedback term Y
	float m_IntegralFBz;       // Integral feedback term Z

	static float invSqrt(float x) {
		return 1.0f / std::sqrt(x);
	}

public:
	/**
	 * @brief  Constructor — invalid inputs are silently corrected
	 */
	Mahony(float kp = 0.5f, float ki = 0.1f, float sampleFrq = 200.0f)
		: m_q{1.0f, 0.0f, 0.0f, 0.0f}
		, m_SampleFrq(sampleFrq > 0.0f ? sampleFrq : 200.0f)
		, m_Kp(std::fabs(kp))
		, m_Ki(std::fabs(ki))
		, m_IntegralFBx(0.0f)
		, m_IntegralFBy(0.0f)
		, m_IntegralFBz(0.0f)
	{
	}

	/**
	 * @brief  Mahony 6-axis AHRS update (gyroscope + accelerometer)
	 * @note   For 9-axis (gyro + accel + mag), use Update9Axis (planned)
	 * @param  gx/gy/gz: gyroscope readings in rad/s
	 * @param  ax/ay/az: accelerometer readings in g
	 */
	void Update6Axis(float gx, float gy, float gz,
	                 float ax, float ay, float az)
	{
		float detT = 1.0f / m_SampleFrq;

		/* Normalize accelerometer measurement */
		float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		/* Get the gravity in the coordinate of IMU */
		float vx = 2 * (m_q[1] * m_q[3] - m_q[0] * m_q[2]);
		float vy = 2 * (m_q[2] * m_q[3] + m_q[0] * m_q[1]);
		float vz = 1 - 2 * (m_q[1] * m_q[1] + m_q[2] * m_q[2]);

		/* Cross product to get the gravity error between the gyroscope and accelerometer */
		/* a x v */
		float errX = ay * vz - az * vy;
		float errY = az * vx - ax * vz;
		float errZ = ax * vy - vx * ay;

		/* PI control: per-axis integral accumulation */
		m_IntegralFBx += m_Ki * errX * detT;
		m_IntegralFBy += m_Ki * errY * detT;
		m_IntegralFBz += m_Ki * errZ * detT;

		/* Corrected angular velocity = gyro + PI correction */
		float wx = gx + (m_Kp * errX + m_IntegralFBx);
		float wy = gy + (m_Kp * errY + m_IntegralFBy);
		float wz = gz + (m_Kp * errZ + m_IntegralFBz);

		/* Differential quaternion update */
		m_q[0] = m_q[0] + 0.5f*detT * (-wx * m_q[1] - wy * m_q[2] - wz * m_q[3]);
		m_q[1] = m_q[1] + 0.5f*detT * ( wx * m_q[0] - wy * m_q[3] + wz * m_q[2]);
		m_q[2] = m_q[2] + 0.5f*detT * ( wx * m_q[3] + wy * m_q[0] - wz * m_q[1]);
		m_q[3] = m_q[3] + 0.5f*detT * (-wx * m_q[2] + wy * m_q[1] + wz * m_q[0]);

		/* Normalize quaternion */
		recipNorm = invSqrt(m_q[0] * m_q[0] + m_q[1] * m_q[1] + m_q[2] * m_q[2] + m_q[3] * m_q[3]);
		m_q[0] *= recipNorm;
		m_q[1] *= recipNorm;
		m_q[2] *= recipNorm;
		m_q[3] *= recipNorm;
	}

	/**
	 * @brief  Convert quaternion to Euler angles
	 * @param  pitch/yaw/roll: output Euler angles in radians
	 */
	void GetEulerAngle(float* pitch, float* yaw, float* roll)
	{
		/* Roll (X-axis) */
		float sinr_cosp = 2 * (m_q[0] * m_q[1] + m_q[2] * m_q[3]);
		float cosr_cosp = 1 - 2 * (m_q[1] * m_q[1] + m_q[2] * m_q[2]);
		*roll = std::atan2(sinr_cosp, cosr_cosp);

		/* Pitch (Y-axis) */
		float sinp = 2 * (m_q[0] * m_q[2] - m_q[3] * m_q[1]);
		if (std::fabs(sinp) >= 1.0f)
			*pitch = std::copysign(3.1415926535898f / 2.0f, sinp);
		else
			*pitch = std::asin(sinp);

		/* Yaw (Z-axis) */
		float siny_cosp = 2 * (m_q[0] * m_q[3] + m_q[1] * m_q[2]);
		float cosy_cosp = 1 - 2 * (m_q[2] * m_q[2] + m_q[3] * m_q[3]);
		*yaw = std::atan2(siny_cosp, cosy_cosp);
	}

	/**
	 * @brief  Reset filter state (quaternion → identity, integrals → zero)
	 */
	void reset() {
		m_q[0] = 1.0f; m_q[1] = 0.0f; m_q[2] = 0.0f; m_q[3] = 0.0f;
		m_IntegralFBx = 0.0f;
		m_IntegralFBy = 0.0f;
		m_IntegralFBz = 0.0f;
	}
};

#endif /* __MAHONY_H */
