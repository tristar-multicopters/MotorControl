/**
  ******************************************************************************
  * @file    pedal_assist.c
  * @author  FTEX inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal assist
  ******************************************************************************
*/

#include "pedal_speed_sensor.h"

// ==================== Private function prototypes ======================== //

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
