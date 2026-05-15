/**
  ******************************************************************************
  * @file    pid_drv.h
  * @brief   C++ PID Controller Class Header File
  ******************************************************************************
  */

/*
 * File Name        : pid_drv.h
 * Description      : C++ PID controller class
 * Target Platform  : STM32F1/F4/F7 Series
 * Dependencies     : Header file for C++ PID controller class
 * Author           : WU Yandong(Mark)
 * Last Updated     : 2026-03-30
 *
 * Team Notes:
 * Before you modify the code, make sure that you understand your code and modified function
 * 
 */

#ifndef __PID_DRV_H
#define __PID_DRV_H

#include <cmath> 

class PID {
private:
	
public:
	float m_Kp;
	float m_Ki; 
	float m_Kd;
	float m_Kf1;	// Larger Kf1 higher flexibility
	float m_Kf2;	// Larger Kf2 higher damping

	float m_target;
	float m_last_target;
	float m_measure;

	float m_error;
	float m_last_error;

	float m_p_term;
	float m_i_term;
	float m_d_term;
	float m_f_term;

	float m_integral_limit;// This number must larger than 0
	float m_output_limit;// This number must larger than 0
	
	float m_output;
	float m_dead_band;

	PID(float kp, float ki, float kd, float integral_limit, float output_limit)
	: m_Kp(kp), m_Ki(ki), m_Kd(kd), m_Kf1(0), m_Kf2(0),
	  m_target(0), m_measure(0), m_error(0), m_last_error(0),
	  m_p_term(0), m_i_term(0), m_d_term(0), m_f_term(0),
	  m_integral_limit(integral_limit), m_output_limit(output_limit),
	  m_output(0), m_dead_band(0)
	{
	}

	void setTarget(float new_target){
		m_last_target = m_target;
		m_target = new_target;
	}
	
	void setMeasure(float measure){
		m_measure = measure;
	}

	float calculate()
	{	
		m_error = m_target - m_measure;
		
		if (std::fabsf(m_error) < m_dead_band)
		{
			m_last_error = m_error;
			m_i_term = 0;
			return 0.0f;
		}
   
		m_p_term = m_Kp * m_error;
		m_i_term += m_Ki * m_error;
		m_d_term = m_Kd * (m_error - m_last_error);
		m_f_term = m_Kf1 * m_target - m_Kf2 * m_last_target;
		
		// Integral limitation
		if (m_i_term > m_integral_limit)
		{
			m_i_term = m_integral_limit;
		} 
		else if (m_i_term < -m_integral_limit)
		{
			m_i_term = -m_integral_limit;
		}
		
		m_output = m_p_term + m_i_term + m_d_term + m_f_term;
		
		// output limitation
		if (m_output > m_output_limit)
		{
			m_output = m_output_limit;
		}
		else if (m_output < -m_output_limit)
		{
			m_output = -m_output_limit;
		}
		
		m_last_error = m_error;
		m_last_target = m_target;
		return m_output;
	}

	void reset()
	{
		m_target = 0;
		m_measure = 0;
		m_error = 0;
		m_last_error = 0;
		m_p_term = 0;
		m_i_term = 0;
		m_d_term = 0;
		m_output = 0;
	}
	
	PID& operator=(const PID& other)
	{
		m_Kp = other.m_Kp;
		m_Ki = other.m_Ki;
		m_Kd = other.m_Kd;
		m_integral_limit = std::fabsf(other.m_integral_limit);
		m_output_limit = std::fabsf(other.m_output_limit);
		return *this;
	}
	
	void SetDeadBand(float deadband)
	{
		m_dead_band = deadband;
	}
	
	void SetFeedForward(float flexibility, float smoothness)
	{
		m_Kf1 = flexibility;
		m_Kf2 = smoothness;
	}
};

#endif
