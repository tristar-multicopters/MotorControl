/**
  ******************************************************************************
  * @file    pedal_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes
  *           used in higher level modules for pedal speed sensor
  *
	******************************************************************************
*/


#ifndef __PEDAL_SPEED_SENSOR_H
#define __PEDAL_SPEED_SENSOR_H


#include "pulse_frequency.h"


typedef struct {
    PulseFrequencyHandle_t * pPulseFrequency;    /* Pointer to speed handle */

    uint8_t	bPulsePerRotation;   		         /* Number of pulse per rotation */

	int32_t wSpeedRPM;   						 /* Pedal Speed sensor RPM calculated value */
    bool bSpeedDetected;     			 	     /* True if speed is detected */

} PedalSpeedSensorHandle_t;


// ==================== Public function prototypes ========================= //

/**
  @brief  Function to initialize pedal speed sensor handle
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_Init(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to return speed in rpm
  @param  PedalSpeedSensorHandle_t handle
  @retval Speed in rpm
*/
int32_t PedalSpdSensor_GetSpeedRPM(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to check if speed is detected
  @param  WheelSpeedSensorHandle_t handle
  @return True if speed is detected
*/
bool PedalSpdSensor_IsSpeedDetected(PedalSpeedSensorHandle_t * pHandle);


#endif
