/**
  * @file    throttle.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
*/

#include "throttle.h"


/* Functions ---------------------------------------------------- */


void THRO_Init(THRO_Handle_t * pHandle)
{
	/* Need to be register with RegularConvManager */
//	pHandle->bConvHandle = RCM_AddConv(pHandle->pRegularConversionManager, pHandle->hChannelConfig);
	THRO_Clear( pHandle );
}


void THRO_Clear(THRO_Handle_t * pHandle)
{
	pHandle->hAvADCValue = 0u;
  pHandle->hAvThrottleValue = 0u;
}


void THRO_CalcAvThrottleValue(THRO_Handle_t * pHandle)
{
	uint32_t wAux;
  uint16_t hAux;
	uint16_t hBandwidth;
	
	/*
		Compute averaged raw ADC value (between 0 and 65535)
	*/
//	hAux = RCM_ReadConv(pHandle->pRegularConversionManager, pHandle->bConvHandle);
	pHandle->hInstADCValue = hAux;
	
	if (pHandle->hInstADCValue > pHandle->hAvADCValue)
		hBandwidth = pHandle->hParam.hLowPassFilterBW1;
	else
		hBandwidth = pHandle->hParam.hLowPassFilterBW2;

	if ( hAux != UINT16_MAX )
	{
		wAux =  ( uint32_t )( hBandwidth - 1u );
		wAux *= ( uint32_t ) ( pHandle->hAvADCValue );
		wAux += hAux;
		wAux /= ( uint32_t )( hBandwidth );

		pHandle->hAvADCValue = ( uint16_t ) wAux;
	}
	
	/*
		Compute throttle value (between 0 and 65535)
	*/
	hAux = (pHandle->hAvADCValue > pHandle->hParam.hOffsetThrottle) ? 
					(pHandle->hAvADCValue - pHandle->hParam.hOffsetThrottle) : 0; //Substraction without overflow
	
	wAux = (uint32_t)(pHandle->hParam.bSlopeThrottle * hAux);
	wAux /= pHandle->hParam.bDivisorThrottle;
	if (wAux > UINT16_MAX)
		wAux = UINT16_MAX;
	hAux = (uint16_t)wAux;
	
	pHandle->hAvThrottleValue = hAux;
}


uint16_t THRO_GetAvThrottleValue(THRO_Handle_t * pHandle)
{
  return ( pHandle->hAvThrottleValue );
}


int16_t THRO_ThrottleToTorque(THRO_Handle_t * pHandle)
{
	int32_t wAux;
	
	/*
		Compute torque value (between -32768 and 32767)
	*/
	wAux = pHandle->hAvThrottleValue - pHandle->hParam.hOffsetTorque;
	if (wAux < 0)
		wAux = 0;
	
	wAux = (int32_t)(pHandle->hParam.bSlopeTorque * wAux);
	wAux /= pHandle->hParam.bDivisorTorque;
	if (wAux > INT16_MAX)
		wAux = INT16_MAX;
	else if (wAux < INT16_MIN)
		wAux = INT16_MIN;
			
	return (int16_t)wAux;
}

int16_t THRO_ThrottleToSpeed(THRO_Handle_t * pHandle)
{
	// todo: implementation
	
	// return dummy value so compiler doesn't complain
	return -1;
}


bool THRO_IsThrottleDetected (THRO_Handle_t * pHandle) 
{
	uint16_t hThrottle;
	hThrottle = THRO_GetAvThrottleValue(pHandle);
	if (hThrottle <= pHandle->hParam.hDetectionThreshold)
		return false;
	else 
		return true;	
}


