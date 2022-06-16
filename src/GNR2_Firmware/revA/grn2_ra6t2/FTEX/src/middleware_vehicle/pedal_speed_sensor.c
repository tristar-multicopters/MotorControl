/**
  ******************************************************************************
  * @file    pedal_speed_sensor.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal speed sensor
  ******************************************************************************
*/

#include "pedal_speed_sensor.h"

// ==================== Private function prototypes ======================== //

/**
    Pedal Speed Sensor module Initialization
*/
void PAS_Init(PAS_Handle_t* pHandle)
{
    PedalTorqSensor_Init(pHandle->pTorque);
}

/**
    Pedal Speed Sensor average value calculation
*/
void PAS_CalcTSAvValue(PAS_Handle_t* pHandle)
{
    PedalTorqSensor_CalcAvValue(pHandle->pTorque);
}

/**
	Pedal Assist capture Periode calculation
*/
void PAS_CalculateSpeed(PAS_Handle_t* pHandle)
{	
    PulseFrequency_ReadInputCapture_AGT (pHandle->pSpulse); 
}

/**
	Pedal Assist Periode Get value
*/
uint32_t PAS_GetPeriodValue(PAS_Handle_t* pHandle)
{	
    return pHandle->wPedal_Sensor_Read;
}
