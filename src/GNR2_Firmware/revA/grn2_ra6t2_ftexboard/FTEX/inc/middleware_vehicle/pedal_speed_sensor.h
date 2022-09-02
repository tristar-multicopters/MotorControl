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

// =============================== Includes ================================== //
#include "pulse_frequency.h"
#include "pedal_torque_sensor.h"

// ================= Structure used to configure a pin ===================== //
typedef struct {
	PulseFrequencyHandle_t * pPulseFrequency;   /* Pointer to pedal handle */

	uint32_t wPedalSpeedSens_Freq;  /* Pedal Speed sensor frequency calculated value */
	uint32_t wPedalSpeedSens_Period;    /* Pedal Speed Sensor Periode value*/
	int32_t  wPedalSpeedSens_RPM;   /* Pedal Speed sensor RPM calculated value */

	uint8_t	bPulsePerRotation;			/* Number of pulse per rotation */

} PedalSpeedSensorHandle_t;

// ==================== Public function prototypes ========================= //
/**
  @brief  Function to initialize pedal speed sensor handle
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_Init(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to capture pedal speed sensor Periode
  @param  PedalSpeedSensorHandle_t handle
  @return None
*/
void PedalSpdSensor_CalculateSpeed(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to Get the Pedal Speed Sensor Periode
  @param  PedalSpeedSensorHandle_t handle
  @return wPedal_Sensor_Read in unit32_t
*/
uint32_t PedalSpdSensor_GetPeriodValue(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to Get the Pedal Speed Sensor Frequency
  @param  PedalSpeedSensorHandle_t handle
  @return wPedalSpeedSens_Freq in unit32_t
*/
uint32_t PedalSpdSensor_GetSpeedFreq(PedalSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to return speed in rpm
  @param  PedalSpeedSensorHandle_t handle
  @retval Speed in rpm
*/
int32_t PedalSpdSensor_GetSpeedRPM(PedalSpeedSensorHandle_t* pHandle);

#endif
