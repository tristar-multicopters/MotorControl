/**
  ******************************************************************************
  * @file    wheel_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes 
  *          used in higher level modules for wheel speed sensor
  *
  ******************************************************************************
*/


#ifndef __WHEEL_SPEED_SENSOR_H
#define __WHEEL_SPEED_SENSOR_H


#include "pulse_frequency.h"


typedef struct {
	PulseFrequencyHandle_t * pPulseFrequency;

	uint8_t	bPulsePerRotation;    		/* Nunber of pulse per rotation */
		
	int32_t wSpeedRPM;    			    /* Wheel Speed sensor rotation per minute calculated value */
	bool bSpeedDetected;				/* True if speed is detected */
     
} WheelSpeedSensorHandle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Function to initialize wheel speed sensor handle
  @param  WheelSpeedSensorHandle_t handle
  @return None
*/
void WheelSpdSensor_Init(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to return speed in rpm
  @param  WheelSpeedSensorHandle_t handle
  @return Speed in rpm
*/
int32_t WheelSpdSensor_GetSpeedRPM(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to check if speed is detected
  @param  WheelSpeedSensorHandle_t handle
  @return True if speed is detected
*/
bool WheelSpdSensor_IsSpeedDetected(WheelSpeedSensorHandle_t * pHandle);


#endif

