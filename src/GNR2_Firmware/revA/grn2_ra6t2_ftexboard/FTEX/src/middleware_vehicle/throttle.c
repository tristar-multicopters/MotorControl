/**
  * @file    throttle.c
  * @brief   This module handles throttle management
  *
*/

#include "throttle.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
   Initializes throttle sensing conversions
 */
void Throttle_Init(ThrottleHandle_t * pHandle, Delay_Handle_t * pThrottleStuckDelay)
{
    ASSERT(pHandle != NULL);
    
    pHandle->DisableThrottleOutput = false;
        
    pHandle->pThrottleStuckDelay = pThrottleStuckDelay;
      
    ASSERT(pHandle->pThrottleStuckDelay->DelayInitialized); // Delay sohuld be initialized in the task to specify at which 
                                                            // frequence the update delay function will eb called
    
    Delay_SetTime(pHandle->pThrottleStuckDelay, 5, SEC); // Set a 5 seconds delay to detect a stuck throttle
    Delay_Reset(pHandle->pThrottleStuckDelay);           // Make sure the counter is reset 
    
    SignalFiltering_Init(&pHandle->ThrottleFilter);
    SignalFiltering_ConfigureButterworthFOLP(&pHandle->ThrottleFilter,
                                              pHandle->hParameters.fFilterAlpha,
                                              pHandle->hParameters.fFilterBeta);
    
    pHandle->SafeStart = false;
    Foldback_Init(pHandle->SpeedFoldbackVehicleThrottle);
    
    Throttle_SetMaxSpeed(pHandle,pHandle->hParameters.DefaultMaxThrottleSpeedRPM);
    
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

static uint16_t SafeStartCounter = 0;
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

	static bool ThrottleStuck = false;
    
    if(pHandle->DisableThrottleOutput) // Test if we want to disable the throttle on PAS 0
    {
        hAux = 0;  // We are in PAS level 0 so the throttle is disabled
    } 
    else
    {    
        /* Compute averaged raw ADC value (between 0 and 65535) */
        hAux = RegConvMng_ReadConv(pHandle->bConvHandle);
        pHandle->hInstADCValue = hAux;
        pHandle->hAvADCValue = SignalFiltering_CalcOutputU16(&pHandle->ThrottleFilter, hAux);
        
        /* Compute throttle value (between 0 and 65535)  */
        hAux = (pHandle->hAvADCValue > pHandle->hParameters.hOffsetThrottle) ? 
                        (pHandle->hAvADCValue - pHandle->hParameters.hOffsetThrottle) : 0; //Substraction without overflow
        
        wAux = (uint32_t)(pHandle->hParameters.bSlopeThrottle * hAux);
        wAux /= pHandle->hParameters.bDivisorThrottle;
        if (wAux > UINT16_MAX)
        {    
            wAux = UINT16_MAX;
        }
          
        hAux = (uint16_t)wAux;
    }
    
	pHandle->hAvThrottleValue = hAux;
    
    // Throttle stuck on startup verification
    if(!pHandle->SafeStart && !pHandle->DisableThrottleOutput) 
    {   
        if(!Throttle_IsThrottleDetected(pHandle)) // If we haven't done the safe start and throttle is not detected 
        { 
            SafeStartCounter ++; // Increase the counter

            if(SafeStartCounter >= 20) // If we haven't receive a throttle for the 20th time in a row consider the safe start done
            {
                pHandle->SafeStart = true;
                VC_Errors_ClearError(THROTTLE_STUCK); // Clear this error in case it was falsly flagged as stuck (user kept throttle at max on boot)
                Delay_Reset(pHandle->pThrottleStuckDelay);
            }
            else
            {
                pHandle->hAvThrottleValue = 0; // If we still aren't done with the safe start make sure we aren't requesting power
            }            
        }
        else     // Throttle is detected 
        {
            SafeStartCounter = 0;
            pHandle->hAvThrottleValue = 0; // Make sure we don't send power
            
            if(!ThrottleStuck)
            {
                if(Delay_Update(pHandle->pThrottleStuckDelay)) // Increase the counter for the error delay and check if the delay has been reached
                {
                    VC_Errors_RaiseError(THROTTLE_STUCK);
                    ThrottleStuck = true;
                }
            }
        }    
    }
    
    
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

/**
    Set the value of the flag to disable throttle output 
  */
void Throttle_DisableThrottleOutput(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    pHandle->DisableThrottleOutput = true;
}

/**
    Reset the value of the flag to disable throttle output 
  */
void Throttle_EnableThrottleOutput(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    pHandle->DisableThrottleOutput = false;
}

/**
    Set the max speed in RPM that you can reach with throttle 
  */
void Throttle_SetMaxSpeed(ThrottleHandle_t * pHandle, uint16_t aMaxSpeedRPM)
{     
    ASSERT(pHandle != NULL);
     
    ASSERT(pHandle->hParameters.ThrottleDecreasingRange > 0);
    
    Foldback_SetDecreasingRange (pHandle->SpeedFoldbackVehicleThrottle,pHandle->hParameters.ThrottleDecreasingRange);

    if (aMaxSpeedRPM < pHandle->hParameters.MaxSafeThrottleSpeedRPM)
    {
        ASSERT(((int16_t) aMaxSpeedRPM) > 0); // Make sure the cast doesn't result in a negative value
        
        Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackVehicleThrottle, (int16_t) aMaxSpeedRPM); // 240 == 32 km/h  74 = 10 km/h   
    }
    else
    {
        Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackVehicleThrottle, (int16_t) pHandle->hParameters.MaxSafeThrottleSpeedRPM);
    }        
     
}
