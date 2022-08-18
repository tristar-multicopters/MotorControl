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


void RotorPosObs_Init(RotorPositionObserverHandle_t * pHandle)
{
	pHandle->hKpGain = pHandle->hKpGainDef;
	pHandle->hKdGain = pHandle->hKdGainDef;
	pHandle->hKiGain = pHandle->hKiGainDef;
	
	return;
}

void RotorPosObs_Clear(RotorPositionObserverHandle_t * pHandle)
{	
	pHandle->hEstElAngle = pHandle->pHallSensor->Super.hElAngle;
	pHandle->hEstElSpeedDpp = pHandle->pHallSensor->Super.hElSpeedDpp;
	pHandle->hEstMechTorque = 0;

	pHandle->wEstElAngle = pHandle->pHallSensor->Super.hElAngle*INT16_MAX;
	pHandle->wEstElSpeedDpp = pHandle->pHallSensor->Super.hElSpeedDpp*INT16_MAX;
	pHandle->wEstMechTorque = 0;
	
	return;
}

int16_t RotorPosObs_CalcElAngle(RotorPositionObserverHandle_t * pHandle, int16_t hElTorque)
{
	int64_t dAux; int32_t wAux;
	int16_t hErrorSin;
	int32_t wEstElAngle_Next, wEstElSpeed_Next, wEstMechTorque_Next;
	
	hErrorSin = MCMath_TrigFunctions(pHandle->pHallSensor->Super.hElAngle - pHandle->hEstElAngle).hSin;
	
	wAux = pHandle->hKdGainDef;
	if (wAux > INT16_MAX)
		wAux = INT16_MAX;
	if (wAux < -INT16_MAX)
		wAux = -INT16_MAX;
	pHandle->hKdGain = (int16_t) wAux;
	wAux = (pHandle->hKdGain * hErrorSin) >> pHandle->hKdDivisorPOW2;
	wEstElAngle_Next = pHandle->wEstElAngle +	pHandle->wEstElSpeedDpp + wAux;
	
	int32_t wElTorque = hElTorque * INT16_MAX;
	wAux = (pHandle->hKpGain * hErrorSin) >> pHandle->hKpDivisorPOW2;
	dAux = pHandle->wEstElSpeedDpp + (wElTorque - pHandle->wEstMechTorque) + wAux;
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
	pHandle->wEstElSpeedDpp = wEstElSpeed_Next;
	pHandle->wEstMechTorque = wEstMechTorque_Next;
	
	pHandle->hEstElAngle = (int16_t)(pHandle->wEstElAngle / INT16_MAX);
	pHandle->hEstElSpeedDpp =(int16_t)(pHandle->wEstElSpeedDpp / INT16_MAX);
	pHandle->hEstMechTorque = (int16_t)(pHandle->wEstMechTorque / INT16_MAX);
	
	pHandle->Super.hElAngle = pHandle->hEstElAngle;
	pHandle->Super.hElSpeedDpp = pHandle->hEstElSpeedDpp;
	
	return pHandle->hEstElAngle;
}


bool RotorPosObs_CalcMecSpeedUnit(RotorPositionObserverHandle_t * pHandle, int16_t * pMecSpeedUnit)
{
	bool bIsReliable = SpdPosFdbk_GetReliability(&pHandle->pHallSensor->Super);
	bIsReliable &= SpdPosFdbk_CalcReliability (&pHandle->Super, pMecSpeedUnit);
	
	if (bIsReliable)
	{
		/* Convert el_dpp to MecUnit */
		*pMecSpeedUnit = (int16_t)(( pHandle->hEstElSpeedDpp * (int32_t)pHandle->Super.hMeasurementFrequency * (int32_t) SPEED_UNIT) /
																((int32_t) pHandle->Super.DPPConvFactor * (int32_t)pHandle->Super.bElToMecRatio));
	}
	else
	{
		*pMecSpeedUnit = 0;
	}
	
	pHandle->Super.hAvrMecSpeedUnit = *pMecSpeedUnit;
	
	return bIsReliable;
}


int16_t RotorPosObs_GetElAngle(RotorPositionObserverHandle_t * pHandle)
{
	return pHandle->hEstElAngle;
}

int16_t RotorPosObs_GetElSpeed(RotorPositionObserverHandle_t * pHandle)
{
	return pHandle->hEstElSpeedDpp;
}

