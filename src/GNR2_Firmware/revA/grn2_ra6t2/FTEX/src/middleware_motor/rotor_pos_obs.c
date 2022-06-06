/**
  * @file       rotor_pos_obs.c
  * @brief     	This file provides firmware functions that implement the features
  *               of the Angle Observer component .
  *
 **/
 
/* Includes ------------------------------------------------------------------*/

#include "rotor_pos_obs.h"
#include "mc_math.h"

#include "mc_type.h"


void RotorPosObs_Init( RotorPositionObserverHandle_t * pHandle )
{
	pHandle->hKpGain = pHandle->hKpGainDef;
	pHandle->hKdGain = pHandle->hKdGainDef;
	pHandle->hKiGain = pHandle->hKiGainDef;
	
	return;
}

void RotorPosObs_Clear( RotorPositionObserverHandle_t * pHandle )
{	
	pHandle->hEstElAngle = pHandle->pHallFdbk->Super.hElAngle;
	pHandle->hEstElSpeed = pHandle->pHallFdbk->Super.hElSpeedDpp;
	pHandle->hEstMechTorque = 0;

	pHandle->wEstElAngle = pHandle->pHallFdbk->Super.hElAngle*INT16_MAX;
	pHandle->wEstElSpeed = pHandle->pHallFdbk->Super.hElSpeedDpp*INT16_MAX;
	pHandle->wEstMechTorque = 0;
	
	return;
}

int16_t RotorPosObs_CalcElAngle( RotorPositionObserverHandle_t * pHandle, int16_t hElTorque)
{
	int64_t dAux; int32_t wAux;
	int16_t hErrorSin;
	int32_t wEstElAngle_Next, wEstElSpeed_Next, wEstMechTorque_Next;
	
	hErrorSin = MCMath_TrigFunctions( pHandle->pHallFdbk->Super.hElAngle - pHandle->hEstElAngle ).hSin;
	
	wAux = pHandle->hKdGainDef;
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

int16_t RotorPosObs_GetElAngle( RotorPositionObserverHandle_t * pHandle)
{
	return pHandle->hEstElAngle;
}

int16_t RotorPosObs_GetElSpeed( RotorPositionObserverHandle_t * pHandle)
{
	return pHandle->hEstElSpeed;
}

