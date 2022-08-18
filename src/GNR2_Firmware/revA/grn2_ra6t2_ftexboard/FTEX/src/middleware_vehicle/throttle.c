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
void Throttle_Init(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    SignalFiltering_Init(&pHandle->ThrottleFilter);
    SignalFiltering_ConfigureButterworthFOLP(&pHandle->ThrottleFilter,
                                                pHandle->hParameters.fFilterAlpha,
                                                    pHandle->hParameters.fFilterBeta);
    
	/* Need to be register with RegularConvManager */
	pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->Throttle_RegConv);
	Throttle_Clear(pHandle);
}

/**
   Initializes internal average throttle computed value
 */
void Throttle_Clear(ThrottleHandle_t * pHandle)
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
void Throttle_CalcAvThrottleValue(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	uint32_t wAux;
    uint16_t hAux;
	
	/*
		Compute averaged raw ADC value (between 0 and 65535)
	*/
	hAux = RegConvMng_ReadConv(pHandle->bConvHandle);
	pHandle->hInstADCValue = hAux;
    pHandle->hAvADCValue = SignalFiltering_CalcOutputU16(&pHandle->ThrottleFilter, hAux);
    
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
uint16_t Throttle_GetAvThrottleValue(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hAvThrottleValue;
}

/**
    Compute motor torque reference value from current throttle value stored in the handle 
  */
int16_t Throttle_ThrottleToTorque(ThrottleHandle_t * pHandle)
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
int16_t Throttle_ThrottleToSpeed(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	// todo: implementation
	
	// return dummy value so compiler doesn't complain
	return -1;
}

/**
	 Return true if throttled is pressed (threshold is passed) 
   */
bool Throttle_IsThrottleDetected (ThrottleHandle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
	uint16_t hThrottle;
	hThrottle = Throttle_GetAvThrottleValue(pHandle);
	if (hThrottle <= pHandle->hParameters.hDetectionThreshold)
    {    
		return false;
    }    
	else
    {        
		return true;
    }   
}


