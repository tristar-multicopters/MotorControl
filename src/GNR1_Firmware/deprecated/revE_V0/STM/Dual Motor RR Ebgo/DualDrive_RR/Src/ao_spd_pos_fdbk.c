/**
  ******************************************************************************
  * @file       ao_spd_pos_fdbk.c
  * @author  		Ronak Nemade, FTEX.Inc
	@author  			Sami Bouzid, FTEX.Inc
  * @brief     	This file provides firmware functions that implement the features
  *               of the Angle Observer  component  for the  Motor Control SDK.
  *
  ******************************************************************************
 **/
 
/* Includes ------------------------------------------------------------------*/

#include "ao_spd_pos_fdbk.h"
#include "mc_math.h"

#include "mc_type.h"

/**
  * @brief  It initializes the state observer object
  * @param  pHandle: handler of the current instance of the angle observer component
  * @retval none
  */
__weak void AO_Init( AO_Handle_t * pHandle )
{
	pHandle->hKpGain = pHandle->hKpGainDef;
	pHandle->hKdGain = pHandle->hKdGainDef;
	pHandle->hKiGain = pHandle->hKiGainDef;
	
	return;
}

/* It clears state observer object by re-initializing private variables*/
__weak void AO_Clear( AO_Handle_t * pHandle )
{	
	pHandle->hEstElAngle = pHandle->pHallFdbk->_Super.hElAngle;
	pHandle->hEstElSpeed = pHandle->pHallFdbk->_Super.hElSpeedDpp;
	pHandle->hEstMechTorque = 0;

	pHandle->wEstElAngle = pHandle->pHallFdbk->_Super.hElAngle*INT16_MAX;
	pHandle->wEstElSpeed = pHandle->pHallFdbk->_Super.hElSpeedDpp*INT16_MAX;
	pHandle->wEstMechTorque = 0;
	
	return;
}

__weak int16_t AO_CalcElAngle( AO_Handle_t * pHandle, int16_t hElTorque)
{
	int64_t dAux; int32_t wAux; int16_t hAux;
	int16_t hErrorSin;
	int32_t wEstElAngle_Next, wEstElSpeed_Next, wEstMechTorque_Next;
	
	hErrorSin = MCM_Trig_Functions( pHandle->pHallFdbk->_Super.hElAngle - pHandle->hEstElAngle ).hSin;
	
	wAux = ( abs(pHandle->hEstElSpeed) * pHandle->hSpeedFactorGain ) / pHandle->hSpeedFactorDiv;
	wAux += pHandle->hKdGainDef;
	if (wAux > INT16_MAX)
		wAux = INT16_MAX;
	if (wAux < -INT16_MAX)
		wAux = -INT16_MAX;
	pHandle->hKdGain = (int16_t) wAux;
	wAux = (pHandle->hKdGain * hErrorSin) >> pHandle->hKdDivisorPOW2;
	wEstElAngle_Next = pHandle->wEstElAngle +	pHandle->wEstElSpeed + wAux;
	
	int32_t wElTorque = hElTorque * INT16_MAX;
	wAux = (pHandle->hKpGain * hErrorSin) >> pHandle->hKpDivisorPOW2;
	dAux = pHandle->wEstElSpeed + (wElTorque - pHandle->wEstMechTorque) + wAux;
	if (dAux > INT32_MAX)
		dAux = INT32_MAX;
	if (dAux < -INT32_MAX)
		dAux = -INT32_MAX;
	wEstElSpeed_Next = (int32_t) dAux;
	
	wAux = (pHandle->hKiGain * hErrorSin) >> pHandle->hKiDivisorPOW2;
	dAux = pHandle->wEstMechTorque + wAux;
	if (dAux > INT32_MAX)
		dAux = INT32_MAX;
	if (dAux < -INT32_MAX)
		dAux = -INT32_MAX;
	wEstMechTorque_Next = (int32_t) dAux;

	pHandle->wEstElAngle = wEstElAngle_Next;
	pHandle->wEstElSpeed = wEstElSpeed_Next;
	pHandle->wEstMechTorque = wEstMechTorque_Next;
	
	pHandle->hEstElAngle = pHandle->wEstElAngle / INT16_MAX;
	pHandle->hEstElSpeed = pHandle->wEstElSpeed / INT16_MAX;
	pHandle->hEstMechTorque = pHandle->wEstMechTorque / INT16_MAX;
	
	return pHandle->hEstElAngle;
}

int16_t AO_GetElAngle( AO_Handle_t * pHandle)
{
	return pHandle->hEstElAngle;
}

int16_t AO_GetElSpeed( AO_Handle_t * pHandle)
{
	return pHandle->hEstElSpeed;
}

