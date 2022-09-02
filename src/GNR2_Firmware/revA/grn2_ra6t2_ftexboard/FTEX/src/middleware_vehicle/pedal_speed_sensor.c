/**
  ******************************************************************************
  * @file    pedal_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal speed sensor
  ******************************************************************************
*/

#include "pedal_speed_sensor.h"
#include "ASSERT_FTEX.h"

// ==================== Public function prototypes ======================== //
/**
    Pedal Speed Sensor module Initialization
*/
void PedalSpdSensor_Init(PedalSpeedSensorHandle_t* pHandle)
{
	ASSERT(pHandle != NULL);    
}

/**
	Pedal Speed Sensor capture Periode calculation
*/
void PedalSpdSensor_CalculateSpeed(PedalSpeedSensorHandle_t* pHandle)
{	
	PulseFrequency_ReadInputCapture (pHandle->pPulseFrequency); 
	pHandle->wPedalSpeedSens_Period = (uint32_t) pHandle->pPulseFrequency->wCaptureCount; 
}

/**
	Pedal Speed Sensor Periode Get value
*/
uint32_t PedalSpdSensor_GetPeriodValue(PedalSpeedSensorHandle_t* pHandle)
{	
	return pHandle->wPedalSpeedSens_Period;
}

/**
	Pedal Speed Sensor Frequency Get value
*/
uint32_t PedalSpdSensor_GetSpeedFreq(PedalSpeedSensorHandle_t* pHandle)
{	
	pHandle->wPedalSpeedSens_Freq = COEFFREQ / PedalSpdSensor_GetPeriodValue(pHandle);
	return pHandle->wPedalSpeedSens_Freq;
}

/**
	Pedal Speed Sensor RPM Get value
*/
int32_t PedalSpdSensor_GetSpeedRPM(PedalSpeedSensorHandle_t* pHandle)
{	
	//int8_t 	bdirec = Get_Drive_Direction (pHandle->pSpulse);
	pHandle->wPedalSpeedSens_RPM = (int32_t)((PedalSpdSensor_GetSpeedFreq(pHandle) / pHandle->bPulsePerRotation)* RPMCOEFF);
	return pHandle->wPedalSpeedSens_RPM;
}

