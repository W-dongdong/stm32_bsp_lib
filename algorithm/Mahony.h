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

#define RAD2DGR	(180.0f/3.1415926535898f)

class Mahony {
public:
	float m_q[4];              // Quaternion [w, x, y, z]
	float m_detT;
	float m_Kp;                // Proportional gain
	float m_Ki;                // Integral gain

private:
	float m_IntegralFBx;       // Integral feedback term X
	float m_IntegralFBy;       // Integral feedback term Y
	float m_IntegralFBz;       // Integral feedback term Z

public:
	/**
	 * @brief  Constructor — invalid inputs are silently corrected
	 */
	Mahony(float sampleFrq, float kp = 1.0f, float ki = 0.5f)
		: m_Kp(std::fabs(kp))
		, m_Ki(std::fabs(ki))
		, m_IntegralFBx(0.0f)
		, m_IntegralFBy(0.0f)
		, m_IntegralFBz(0.0f)
	{
		 m_q[0] = 1.0f;
		 m_q[1] = 0.0f;
		 m_q[2] = 0.0f;
		 m_q[3] = 0.0f;

		if (sampleFrq > 0.0f){
			m_detT = 1.0f/sampleFrq;
		} else {
			m_detT = 0;
		}
	}

	/**
	 * @brief  Mahony 6-axis AHRS update (gyroscope + accelerometer)
	 * @param  gx/gy/gz: gyroscope readings in rad/s
	 * @param  ax/ay/az: accelerometer readings in g
	 */
	void SixAxisUpdate(float gx, float gy, float gz,
	                 float ax, float ay, float az)
	{
		/* Normalize accelerometer measurement */
		float accMagnitude = std::sqrt(ax*ax + ay*ay + az*az);
		float recipNorm = 1;

		float wx = gx;
		float wy = gy;
		float wz = gz;

		/* Overweight and weightlessness detect */
		if (accMagnitude>=0.5f && accMagnitude <= 2.0f)
		{
			recipNorm = 1.0f/accMagnitude;

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
			m_IntegralFBx += m_Ki * errX * m_detT;
			m_IntegralFBy += m_Ki * errY * m_detT;
			m_IntegralFBz += m_Ki * errZ * m_detT;	

			/* Corrected angular velocity = gyro + PI correction */
			wx = gx + (m_Kp * errX + m_IntegralFBx);
			wy = gy + (m_Kp * errY + m_IntegralFBy);
			wz = gz + (m_Kp * errZ + m_IntegralFBz);
		}

		/* Differential quaternion update */
		m_q[0] = m_q[0] + 0.5f*m_detT * (-wx * m_q[1] - wy * m_q[2] - wz * m_q[3]);
		m_q[1] = m_q[1] + 0.5f*m_detT * ( wx * m_q[0] - wy * m_q[3] + wz * m_q[2]);
		m_q[2] = m_q[2] + 0.5f*m_detT * ( wx * m_q[3] + wy * m_q[0] - wz * m_q[1]);
		m_q[3] = m_q[3] + 0.5f*m_detT * (-wx * m_q[2] + wy * m_q[1] + wz * m_q[0]);

		/* Normalize quaternion */
		recipNorm = 1.0f/std::sqrt(m_q[0] * m_q[0] + m_q[1] * m_q[1] + m_q[2] * m_q[2] + m_q[3] * m_q[3]);
		m_q[0] *= recipNorm;
		m_q[1] *= recipNorm;
		m_q[2] *= recipNorm;
		m_q[3] *= recipNorm;
	}

	/**
	 * @brief  Convert quaternion to Euler angles
	 * @param  pitch/yaw/roll: output Euler angles in degree
	 */
	uint8_t GetEulerAngle(float* pitch, float* yaw, float* roll)
	{
		float Norm = m_q[0]*m_q[0]+m_q[1]*m_q[1]+m_q[2]*m_q[2]+m_q[3]*m_q[3];
		if (Norm > 0.999f && Norm < 1.001f) {
			*pitch = std::asin(2*(m_q[0]*m_q[2] - m_q[1]*m_q[3]))*RAD2DGR;
			*yaw   = std::atan2(m_q[0]*m_q[3]+m_q[1]*m_q[2], 1-2*(m_q[2]*m_q[2]+m_q[3]*m_q[3]))*RAD2DGR;
			*roll  = std::atan2(m_q[0]*m_q[1]+m_q[2]*m_q[3], 1-2*(m_q[1]*m_q[1]+m_q[2]*m_q[2]))*RAD2DGR;
			return 1;
		}
		return 0;
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
