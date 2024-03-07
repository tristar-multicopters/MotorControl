/**
  * @file    throttle.c
  * @brief   This module handles throttle management
  *
  */

#include "throttle.h"
#include "wheel.h"
#include "ASSERT_FTEX.h"
/* Functions ---------------------------------------------------- */
/**
   Initializes throttle sensing conversions
 */
void Throttle_Init(ThrottleHandle_t * pHandle, Delay_Handle_t * pThrottleStuckDelay)
{
    ASSERT(pHandle != NULL);
    pHandle->DisableThrottleOutput = false;
    pHandle->extThrottleEnable = false;    
    
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
    
    pHandle->CruiseControlEnable = false;
    pHandle->CruiseControlTorqueAvg = 0;
    
    pHandle->hParameters.MaxThrottleSpeedKMH = pHandle->hParameters.DefaultMaxThrottleSpeedKMH; 
    
    /* Need to be register with RegularConvManager */
    pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->Throttle_RegConv);
    Throttle_Clear(pHandle);
    Throttle_ComputeSlopes(pHandle); // Used to calculate values to calibrate ADC value to a standard value
     
    Ramps_Init(&(pHandle->hParameters.ThrottleRamps[0]));
    Ramps_Init(&(pHandle->hParameters.ThrottleRamps[1]));
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
    
    if(pHandle->DisableThrottleOutput || pHandle->BlockOffThrottle) // Test if we want to disable the throttle on PAS 0 or the throttle BlockOff
    {
        hAux = 0;  //The throttle is blocked or We are in PAS level 0 so the throttle is disabled
    } 
    else if(pHandle->extThrottleEnable == false)
    {    
        /* Compute averaged raw ADC value (between 0 and 65535) */
        hAux = RegConvMng_ReadConv(pHandle->bConvHandle);
        pHandle->hInstADCValue = hAux;
        pHandle->hAvADCValue = SignalFiltering_CalcOutputU16(&pHandle->ThrottleFilter, hAux);
        
        /* Compute throttle value (between 0 and 65535)  */
        if (pHandle->hAvADCValue > pHandle->hParameters.hOffsetThrottle)
        {
            hAux = (pHandle->hAvADCValue - pHandle->hParameters.hOffsetThrottle); 
        }
        else
        {
            hAux = 0; 
        }    
                                
        wAux = (uint32_t)(pHandle->hParameters.bSlopeThrottle * hAux);
        wAux /= (uint32_t) pHandle->hParameters.bDivisorThrottle;
        if (wAux > INT16_MAX)
        {    
            wAux = INT16_MAX;
        }
          
        hAux = (uint16_t)wAux;
    }
    else /*if (pHandle->extThrottleEnable == true)*/
    {
        /* Compute averaged external ADC value */
        hAux = pHandle->hExtLatestVal;
        
        /* Compute throttle value (between 0 and 65535)  */
        if (hAux > pHandle->hParameters.hOffsetThrottle)
        {
            hAux = (hAux - pHandle->hParameters.hOffsetThrottle); 
        }
        else
        {
            hAux = 0; 
        }    
                                
        wAux = (uint32_t)(pHandle->hParameters.bSlopeThrottle * hAux);
        wAux /= (uint32_t) pHandle->hParameters.bDivisorThrottle;
        if (wAux > INT16_MAX)
        {    
            wAux = INT16_MAX;
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

            if(SafeStartCounter >= SAFE_THROTTLE_COUNT_100MS) // If we haven't receive a throttle for the 20th time in a row consider the safe start done
            {
                pHandle->SafeStart = true;
                VC_Errors_ClearError(THROTTLE_STUCK); // Clear this error in case it was falsly flagged as stuck (user kept throttle at max on boot)
            }
            else
            {
                pHandle->hAvThrottleValue = 0; // If we still aren't done with the safe start make sure we aren't requesting power
            } 
            
            Delay_Reset(pHandle->pThrottleStuckDelay);              
        }
        else     // Throttle is detected 
        {
            SafeStartCounter = 0;
            pHandle->hAvThrottleValue = 0; // Make sure we don't send power
            
            if(!ThrottleStuck)
            {
                if(Delay_Update(pHandle->pThrottleStuckDelay)) // Increase the counter for the error delay and check if the delay has been reached
                {
                    VC_Errors_RaiseError(THROTTLE_STUCK, HOLD_UNTIL_CLEARED);
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
        Compute torque value (between 0 and 65535)
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
    
    if(pHandle->CruiseControlEnable == true)
    {
        wAux = INT16_MAX;
        
        wAux = Throttle_ApplyCruiseFilter(pHandle,(int16_t)wAux);
    }
                
    return (int16_t) wAux;    
}

/**
   Compute motor speed reference value from current throttle value stored in the handle 
 */
uint16_t Throttle_ThrottleToSpeed(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int32_t wAux;
    
    /*
        Compute speed value (between -32768 and 32767)
    */
    wAux = pHandle->hAvThrottleValue - pHandle->hParameters.hOffsetSpeed;
    if (wAux < 0)
    {
        wAux = 0;
    }
    
    wAux = (int32_t)(pHandle->hParameters.bSlopeSpeed * wAux);
    wAux /= pHandle->hParameters.bDivisorSpeed;
    
    if (wAux > INT16_MAX)
    {    
        wAux = INT16_MAX;
    }    
    else if (wAux < INT16_MIN)
    {    
        wAux = INT16_MIN;
    }        
    return (uint16_t) wAux;
}

/**
   Return true if throttled is pressed (threshold is passed) 
 */
bool Throttle_IsThrottleDetected (ThrottleHandle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
    uint16_t hThrottle;

    hThrottle = Throttle_GetAvThrottleValue(pHandle);
    if ((hThrottle <= pHandle->hParameters.hDetectionThreshold) && (Throttle_GetCruiseControlState(pHandle) != true))
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
   Setup the throttle module to accept an external throttle as the input
 */
void Throttle_SetupExternal(ThrottleHandle_t * pHandle, uint16_t aMaxValue,uint16_t aOffset)
{
   ASSERT(pHandle != NULL);
   ASSERT(aMaxValue > 0);
   ASSERT(aMaxValue > aOffset);
    
   pHandle->extThrottleEnable = true;
    
   pHandle->hParameters.hMaxThrottle = aMaxValue;
   pHandle->hParameters.hOffsetThrottle = aOffset; 

   Throttle_ComputeSlopes(pHandle); 
    
}

/**
   Used to update the value of the throttle, the source of the external throttle should call this function 
 */
void Throttle_UpdateExternal(ThrottleHandle_t * pHandle, uint16_t aNewVal)
{
   ASSERT(pHandle != NULL);
   pHandle->hExtLatestVal = aNewVal;
}

/**
 * Set the max speed in KmH that you can reach with throttle 
 */
void Throttle_SetMaxSpeed(ThrottleHandle_t * pHandle, uint16_t aMaxSpeed)
{
    ASSERT(pHandle != NULL);
    pHandle->hParameters.MaxThrottleSpeedKMH = aMaxSpeed;
}

/**
 * Get the max speed in KmH that you can reach with throttle 
 */
uint16_t Throttle_GetMaxSpeed(ThrottleHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->hParameters.MaxThrottleSpeedKMH;    
}

/**
   Compute slopes for throttle module
 */
void Throttle_ComputeSlopes(ThrottleHandle_t * pHandle)
{
   ASSERT(pHandle != NULL);
   float ADCSlope = 0;
   float Throttle2Torq = 0; 
   float Throttle2Speed = 0;
    
   ADCSlope = (pHandle->hParameters.hMaxThrottle - pHandle->hParameters.hOffsetThrottle); // Calculate the size of usable values received as an input
   
   ASSERT(ADCSlope >= 1); 
   ADCSlope =  INT16_MAX/ADCSlope;       // Calculate the gain needed to scale that value to a 0-int16(32767)
   ADCSlope *= THROTTLE_SLOPE_FACTOR;    // Multiply by the factor to create the numerator of a fraction 
   
   pHandle->hParameters.bSlopeThrottle   = (int16_t) round(ADCSlope);    // Save the numerator
   pHandle->hParameters.bDivisorThrottle = THROTTLE_SLOPE_FACTOR;        // and denominator
    
   // calculate the Slope for Torque 
   Throttle2Torq =  INT16_MAX - pHandle->hParameters.hOffsetTorque;         // Calculate the size of usable values received as an input
   
   ASSERT(Throttle2Torq >= 1); 
   Throttle2Torq =  pHandle->hParameters.ThrottleMaxTorque/Throttle2Torq;   // Calculate the gain needed to scale that value to a 0-hMaxOutputLimitHigh
   Throttle2Torq *= THROTTLE_SLOPE_FACTOR;                                  // Multiply by the factor to create the numerator of a fraction 
   
   pHandle->hParameters.bSlopeTorque   = (int16_t) round(Throttle2Torq);    // Save the numerator
   pHandle->hParameters.bDivisorTorque = THROTTLE_SLOPE_FACTOR;             // and denominator 
    
   // calculate the Slope for Speed 
   Throttle2Speed = INT16_MAX - pHandle->hParameters.hOffsetSpeed;              // Calculate the size of usable values received as an input
   
   ASSERT(Throttle2Speed >= 1); 
   Throttle2Speed =  pHandle->hParameters.MaxThrottleSpeedKMH/Throttle2Speed;   // Calculate the gain needed to scale that value to a 0-hMaxOutputLimitHigh
   Throttle2Speed *= THROTTLE_SLOPE_FACTOR;                                     // Multiply by the factor to create the numerator of a fraction 
    
   pHandle->hParameters.bSlopeSpeed   = (int16_t) round(Throttle2Speed);        // Save the numerator
   pHandle->hParameters.bDivisorSpeed = THROTTLE_SLOPE_FACTOR;                  // and denominator 
}


/**
   Return the cruise control state
 */
bool Throttle_GetCruiseControlState(ThrottleHandle_t * pHandle)
{
     ASSERT(pHandle != NULL);
    
     return pHandle->CruiseControlEnable;
}

/**
   Engage the cruise control feature
 */
void Throttle_EngageCruiseControl(ThrottleHandle_t * pHandle, uint8_t aSpeed)
{
     ASSERT(pHandle != NULL);
    
     if(pHandle->CruiseControlEnable == false)
     {   
        pHandle->CruiseControlTorqueAvg = Throttle_ThrottleToTorque(pHandle);         
        pHandle->CruiseControlEnable = true; 
        Throttle_SetMaxSpeed(pHandle,aSpeed);
     }
}

/**
   Disengage the cruise control feature
 */
void Throttle_DisengageCruiseControl(ThrottleHandle_t * pHandle)
{
     ASSERT(pHandle != NULL);
     
     if(pHandle->CruiseControlEnable == true)
     {
        pHandle->CruiseControlEnable = false; 
        pHandle->CruiseControlTorqueAvg = 0; 
         
        //check if the controller was configured to accept
        //dynamic speed limitation.
        //when no dynamic speed limitation GRN gets deffautl max throttle speed.
        #if DYNAMIC_SPEED_LIMITATION == false
            Throttle_SetMaxSpeed(pHandle,pHandle->hParameters.DefaultMaxThrottleSpeedKMH);
        #endif    
     }
}

/**  
 *  Function used to apply a filter on the Torque when engaging Cruise control
 */
int16_t Throttle_ApplyCruiseFilter(ThrottleHandle_t * pHandle, int16_t aTorque)
{
    ASSERT(pHandle != NULL);
    ASSERT(aTorque >= 0);
    
    const int16_t MaxTorquePerStep = 50; // This represent how much torque we add at every step
    static uint8_t SkipCounter = 0; 
    int16_t TorqueOut = 0;
       
    if(SkipCounter >= 25) // THis represent how spaced appart are the ramp up in throttle torque
    {
        SkipCounter = 0;        
        if(aTorque > (int32_t)(pHandle->CruiseControlTorqueAvg + MaxTorquePerStep))
        { 
            TorqueOut = (pHandle->CruiseControlTorqueAvg + MaxTorquePerStep);
        }       
        else
        {
            TorqueOut = aTorque;
        }
    }
    else
    {
        SkipCounter ++;
        TorqueOut = pHandle->CruiseControlTorqueAvg;
    }    
   
    pHandle->CruiseControlTorqueAvg = TorqueOut;
   
    return TorqueOut;
}

/**
 * @brief  Function used to obtain the ramp of the throttle module
 * @param  pHandle : Pointer on Handle of the throttle, 
 * @retval Ramp to apply
 */
Ramps_Handle_t * Throttle_GetRamp(ThrottleHandle_t * pHandle, uint8_t Direction) 
{
    ASSERT(pHandle != NULL);
    
    return &(pHandle->hParameters.ThrottleRamps[Direction]);
}
