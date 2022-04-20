/**
  ******************************************************************************
  * @file    pedal_assist.c
  * @author  Ronak Nemade, FTEX
  * @brief   This file defines the functions used in higher level modules for pedal assist
  *
	******************************************************************************
*/


#include "pedal_assist.h"


/**
	* @brief  Pedal Assist initialization
	* @param  PAS_Handle_t handle
	* @retval None
	*/
void PAS_Init(PAS_Handle_t* pHandle)
{
	TS_Init(pHandle->pTorque);
	SPR_Init(pHandle->pSpulse);
	SPWR_Init(pHandle->pSpulse);
}

/**
	* @brief  Pedal Assist capture pulse calculation
	* @param  PAS_Handle_t handle
	* @retval None
	*/
void PAS_CalculateSpeed(PAS_Handle_t* pHandle)
{	
	Pedal_capture_get_value( pHandle->pSpulse );
}
/**
	* @brief  Pedal Assist speed Get value
	* @param  PAS_Handle_t handle
	* @retval Pedal sPread value in useconds
	*/
uint32_t PAS_GetPeriodValue(PAS_Handle_t* pHandle)
{	
	return pHandle->pSpulse->sPread;
}

/**
	* @brief  Pedal Assist speed Get Frequency
	* @param  PAS_Handle_t handle
	* @retval Frequency value in Hz
	*/
uint32_t PAS_GetSpeedFreq(PAS_Handle_t* pHandle)
{	
	pHandle->wPASFreq = COEFFREQ / PAS_GetPeriodValue(pHandle);
	return pHandle->wPASFreq;
}

/**
	* @brief  Pedal Assist speed Get RPM
	* @param  PAS_Handle_t handle
	* @retval Pedal sPAvSpeed value in r/min
	*/
int32_t PAS_GetSpeedRPM(PAS_Handle_t* pHandle)
{	
	int8_t 	bdirec = Get_Drive_Direction (pHandle->pSpulse);
	
	pHandle->wPASRpm = ((PAS_GetSpeedFreq(pHandle) / pHandle->bPulseNb)* RPMCOEFF) * bdirec;
	return pHandle->wPASRpm;
}
/**
	* @brief  Pedal Assist capture deirection
	* @param  Forward or back direction
	* @retval Pedal direction in uint8_t
	*/
uint8_t PAS_GetDirection(PAS_Handle_t* pHandle)
{
	return Get_Drive_Direction(pHandle->pSpulse) ;
}

/**
	* @brief  Pedal Assist get torque value
	* @param  PAS_Handle_t handle
	* @retval Torque value
	*/
int16_t PAS_GetTorque(PAS_Handle_t* pHandle)
{
	return TS_CalcAvValue(pHandle->pTorque );
}

/**
	* @brief  Return the PAS Presence Flag
	* @param  Drivetrain handle
	* @retval pHandle->bUsePAS in boolean
	*/
void PAS_UpdatePASDetection (PAS_Handle_t * pHandle) 
{
	uint32_t	pSpeedt;
	PAS_CalculateSpeed(pHandle);
	pSpeedt = PAS_GetPeriodValue(pHandle);
	if (pSpeedt > 0)
		pHandle->bPASDetected = true;
	else 
		pHandle->bPASDetected = false;
} 
