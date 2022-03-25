/**
  ******************************************************************************
  * @file    wheel_speed_sensor.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles torque distribution between motors
  *
	******************************************************************************
*/

#include "wheel_speed_sensor.h"


/* Functions ---------------------------------------------------- */


void WSS_Init(WSS_Handle_t* pHandle)
{
	 SPWR_Init(pHandle->wSpulse);
}

int32_t WSS_GetSpeed(WSS_Handle_t* pHandle)
{
	return Wspeed_CalcAvValue( pHandle->wSpulse);
}

int16_t WSS_GetDirection(WSS_Handle_t* pHandle)
{
	return Get_Drive_Direction(pHandle->wSpulse) ;
}

