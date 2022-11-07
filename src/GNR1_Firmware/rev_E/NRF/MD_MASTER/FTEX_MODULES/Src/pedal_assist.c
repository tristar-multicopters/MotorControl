/**
  ******************************************************************************
  * @file    pedal_assist.c
  * @author  Jabrane Chakroun, FTEX
  * @brief   This file defines the functions used in higher level modules for pedal assist
  *
	******************************************************************************
*/


#include "pedal_assist.h"

/*------------------------------------ Main Functions ---------------------- */
/**
	* @brief  Pedal Assist initialization
	* @param  PAS_Handle_t handle
	* @retval None
	*/
void PAS_Init(PAS_Handle_t* pHandle)
{
	TS_Init(pHandle->pTorque);
	SPR_Init(pHandle->pSpulse);
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
	* @retval Frequency value in mHz
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
	* @brief  Pedal assist torque sensor update
	* @param  PAS_Handle_t handle
	* @retval void
	*/
void PAS_CalcTSAvValue(PAS_Handle_t* pHandle)
{
	TS_CalcAvValue(pHandle->pTorque);
}

/**
	* @brief  Check the PAS Presence Flag
	* @param  Drivetrain handle
	* @retval 
	*/
void PAS_UpdatePASDetection (PAS_Handle_t * pHandle) 
{
	uint32_t	wSpeedt;
	uint16_t	hTorqueSens;
	/* Calculate the offset based on ration percentage */
	uint16_t  hOffsetTemp = (pHandle->pTorque->hParam.hOffsetMT * pHandle->pTorque->hParam.hMax) / MAX_TORQUE_PERCENTAGE;
	
	wSpeedt = PAS_GetPeriodValue(pHandle);
	hTorqueSens = TS_GetAvValue(pHandle->pTorque);

	if ((pHandle->bTorqueSensorUse) && (hTorqueSens > hOffsetTemp) )
		pHandle->bPASDetected = true;
    else if ((pHandle->bHybridSensorUse) && (hTorqueSens > hOffsetTemp) )
		pHandle->bPASDetected = true;
	else if (wSpeedt > 0)
		pHandle->bPASDetected = true;
	else 
		pHandle->bPASDetected = false;
} 

/**
	* @brief  Return if pedals are moving or not
	* @param  Drivetrain handle
	* @retval True if pedal movement is detected, false otherwise
	*/
bool PAS_IsPASDetected(PAS_Handle_t * pHandle) 
{
	return pHandle->bPASDetected;
}

