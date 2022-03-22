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
	 Torque_Init(pHandle->pTorque);
	 SPR_Init(pHandle->pSpulse);
	SPWR_Init(pHandle->pSpulse);
}

/**
	* @brief  Pedal Assist capture pulse length
	* @param  PAS_Handle_t handle
	* @retval Pedal frequency speed
	*/
int32_t PAS_GetSpeed(PAS_Handle_t* pHandle)
{	
	return Pspeed_CalcAvValue( pHandle->pSpulse );
}

/**
	* @brief  Pedal Assist capture pulse length
	* @param  PAS_Handle_t handle
	* @retval Pedal frequency speed
	*/
int32_t PAS_GetSpeedFreq(PAS_Handle_t* pHandle)
{	
	uint32_t PAS_Freq;
	PAS_Freq = Coeff_FREQ / Pspeed_CalcAvValue(pHandle->pSpulse);
	return PAS_Freq;
}

/**
	* @brief  Pedal Assist capture pulse length
	* @param  PAS_Handle_t handle
	* @retval Pedal frequency speed
	*/
int32_t PAS_GetSpeedRPM(PAS_Handle_t* pHandle)
{	
	uint32_t PAS_RPM;
	PAS_RPM = (Coeff_FREQ / Pspeed_CalcAvValue(pHandle->pSpulse))* Coeff_RPM;
	return PAS_RPM;
}

	SPR_Handle_t * pSpulse;				/* Pointer to speed handle */
	SPR_Handle_t * pSpulseRPM;		/* Pointer to speed handle */
	SPR_Handle_t * pSpulseFreq;		/* Pointer to speed handle */


/**
	* @brief  Pedal Assist capture deirection
	* @param  Forward or back direction
	* @retval Pedal direction
	*/
int16_t PAS_GetDirection(PAS_Handle_t* pHandle)
{
	return Get_Drvie_Direction(pHandle->pSpulse) ;
}

/**
	* @brief  Pedal Assist get torque value
	* @param  PAS_Handle_t handle
	* @retval Torque value
	*/
int16_t PAS_GetTorque(PAS_Handle_t* pHandle)
{
	return Torque_CalcAvValue(pHandle->pTorque );
}
