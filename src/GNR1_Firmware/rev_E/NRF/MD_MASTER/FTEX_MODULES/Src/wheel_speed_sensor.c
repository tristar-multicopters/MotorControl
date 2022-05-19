/**
  ******************************************************************************
  * @file    wheel_speed_sensor.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles the Wheel Speed reading 
  *
	******************************************************************************
*/

#include "wheel_speed_sensor.h"


/*------------------------------------ Main Functions ---------------------- */
/**
	* @brief  Wheel Speed Sensor initialization
	* @param  WSS_Handle_t handle
	* @retval None
	*/
void WSS_Init(WSS_Handle_t* pHandle)
{
	 SPWR_Init(pHandle->wSpulse);
}

/**
	* @brief  Wheel Speed Sensor calculation
	* @param  WSS_Handle_t handle
	* @retval None
	*/
void WSS_CalculateSpeed(WSS_Handle_t* pHandle)
{
	Wheel_capture_get_value(pHandle->wSpulse);
}

/**
	* @brief  Wheel Speed Sensor Get value
	* @param  WSS_Handle_t handle
	* @retval Wheel Pulse wPread value in useconds
	*/
int32_t WSS_GetPeriodValue(WSS_Handle_t* pHandle)
{
	return pHandle->wSpulse->wPread;
}

/**
	* @brief  Wheel speed sensor Get Frequency
	* @param  WSS_Handle_t handle
	* @retval Frequency value in mHz
	*/
uint32_t WSS_GetSpeedFreq(WSS_Handle_t* pHandle)
{	
	pHandle->wWSFreq = WCOEFFREQ / WSS_GetPeriodValue(pHandle);
	return pHandle->wWSFreq;
}

/**
	* @brief  Wheel speed senor Calculate RPM
	* @param  WSS_Handle_t handle
	* @retval None
	*/ 
void WSS_CalculateSpeedRPM(WSS_Handle_t* pHandle)
{		
	pHandle->wWSRpm = (((WSS_GetSpeedFreq(pHandle) / pHandle->bWSPulseNb)* WRPMCOEFF)/WPRECISIONCOEFF);
}

/**
	* @brief  Wheel speed senor Get RPM
	* @param  WSS_Handle_t handle
	* @retval Wheel Speed wWSRpm value in r/min
	*/ 
int32_t WSS_GetSpeedRPM(WSS_Handle_t* pHandle)
{
	return pHandle->wWSRpm;
}

/**
	* @brief  Check the WSS Presence Flag
	* @param  WSS_Handle_t handle
	* @retval 
	*/
void WSS_UpdateWSSDetection (WSS_Handle_t * pHandle) 
{
	uint32_t	wWheelSpeed;
	
	wWheelSpeed = WSS_GetSpeedRPM (pHandle);

	if (wWheelSpeed > 0)
		pHandle->bWSSDetected = true;
	else 
		pHandle->bWSSDetected = false;
} 

/**
	* @brief  Return if the wheel speed sensor is moving or not
	* @param  WSS_Handle_t handle
	* @retval True if wheel movement is detected, false otherwise
	*/
bool WSS_IsWSSDetected(WSS_Handle_t * pHandle) 
{
	return pHandle->bWSSDetected;
}
