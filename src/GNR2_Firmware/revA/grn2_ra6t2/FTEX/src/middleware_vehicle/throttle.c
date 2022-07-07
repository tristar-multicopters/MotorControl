/**
  * @file    throttle.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
*/

#include "throttle.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
   Initializes throttle sensing conversions
 */
void THRO_Init(THRO_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
	/* Need to be register with RegularConvManager */
	pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->Throttle_RegConv);
	THRO_Clear(pHandle);
}

/**
   Initializes internal average throttle computed value
 */
void THRO_Clear(THRO_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	pHandle->hAvADCValue = 0u;
    pHandle->hAvThrottleValue = 0u;
}

/**
    Performs the throttle sensing average computation after an ADC conversion.
	Compute torque value in u16 (0 at minimum throttle and 65535 when max throttle).
	Need to be called periodically.
  */
void THRO_CalcAvThrottleValue(THRO_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	uint32_t wAux;
    uint16_t hAux;
	uint16_t hBandwidth;
	
	/*
		Compute averaged raw ADC value (between 0 and 65535)
	*/
	hAux = RegConvMng_ReadConv(pHandle->bConvHandle);
	pHandle->hInstADCValue = hAux;
	
	if (pHandle->hInstADCValue > pHandle->hAvADCValue)
    {    
		hBandwidth = pHandle->hParameters.hLowPassFilterBW1;
	}
    else
    {	
	    hBandwidth = pHandle->hParameters.hLowPassFilterBW2;
    }
    
	if (hAux != UINT16_MAX)
	{
		wAux =  (uint32_t) (hBandwidth - 1u);
		wAux *= (uint32_t) (pHandle->hAvADCValue);
		wAux += hAux;
		wAux /= (uint32_t) (hBandwidth);

		pHandle->hAvADCValue = (uint16_t) wAux;
	}
	
	/*
		Compute throttle value (between 0 and 65535)
	*/
	hAux = (pHandle->hAvADCValue > pHandle->hParameters.hOffsetThrottle) ? 
					(pHandle->hAvADCValue - pHandle->hParameters.hOffsetThrottle) : 0; //Substraction without overflow
	
	wAux = (uint32_t)(pHandle->hParameters.bSlopeThrottle * hAux);
	wAux /= pHandle->hParameters.bDivisorThrottle;
	if (wAux > UINT16_MAX)
    {    
		wAux = UINT16_MAX;
    }   
	hAux = (uint16_t)wAux;
	
	pHandle->hAvThrottleValue = hAux;
}


/**
   Returns latest averaged throttle measured expressed in u16
  */
uint16_t THRO_GetAvThrottleValue(THRO_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hAvThrottleValue;
}

/**
    Compute motor torque reference value from current throttle value stored in the handle 
  */
int16_t THRO_ThrottleToTorque(THRO_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	int32_t wAux;
	
	/*
		Compute torque value (between -32768 and 32767)
	*/
	wAux = pHandle->hAvThrottleValue - pHandle->hParameters.hOffsetTorque;
	if (wAux < 0)
    {
		wAux = 0;
	}
    
	wAux = (int32_t)(pHandle->hParameters.bSlopeTorque * wAux);
	wAux /= pHandle->hParameters.bDivisorTorque;
	
    if (wAux > INT16_MAX)
    {    
		wAux = INT16_MAX;
    }    
	else if (wAux < INT16_MIN)
    {    
		wAux = INT16_MIN;
	}		
	return (int16_t) wAux;
}

/**
    Compute motor speed reference value from current throttle value stored in the handle 
  */
int16_t THRO_ThrottleToSpeed(THRO_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	// todo: implementation
	
	// return dummy value so compiler doesn't complain
	return -1;
}

/**
	 Return true if throttled is pressed (threshold is passed) 
   */
bool THRO_IsThrottleDetected (THRO_Handle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
	uint16_t hThrottle;
	hThrottle = THRO_GetAvThrottleValue(pHandle);
	if (hThrottle <= pHandle->hParameters.hDetectionThreshold)
    {    
		return false;
    }    
	else
    {        
		return true;
    }   
}


