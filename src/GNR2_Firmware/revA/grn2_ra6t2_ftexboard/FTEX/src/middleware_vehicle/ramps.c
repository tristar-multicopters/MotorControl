/**
  * @file    ramps.c
  * @brief   This module handles acceleration/deceleration ramps
  *
*/

#include "ramps.h"
#include <math.h>

/**
  Function used to initialise ramps, calls the proper function for the type of the ramp used
*/
void Ramps_Init(Ramps_Handle_t * pHandle)
{  
    ASSERT(pHandle != NULL);    
    switch(pHandle->RampType)
    {
        case LINEAR:
            Ramps_LinearInit(pHandle);
            break;
        case NO_RAMP:           
        default:
            // Nothing to initialize
        break;   
    }    
}

/**
  Function used to initialise linear ramps
*/
void Ramps_LinearInit(Ramps_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->RampType == LINEAR);
    
    if (pHandle->LinearParameters.Alpha <= 0) // Make sure alpha isn;t 0 as this would break the ramp
    {
        pHandle->LinearParameters.Alpha = 1; 
    }
    
    pHandle->LinearParameters.MaxDelta = pHandle->RampMax/pHandle->LinearParameters.Alpha; // Compute the Max delta
    pHandle->PreviousValue = 0;    
}

/**
  Function used to apply a ramp
*/
uint16_t Ramps_ApplyRamp(Ramps_Handle_t * pHandle, uint16_t Input)
{
    ASSERT(pHandle != NULL);
    uint16_t ValueOut = 0;
    
    static Ramps_Handle_t * LastModifiedRamp = 0;
    
    if (LastModifiedRamp !=  pHandle || LastModifiedRamp == 0) // Did we switch to a different ramp ?
    {
        if (LastModifiedRamp != 0)
        {
           // Transfer the latest value to the new ramp for a smooth transition
           pHandle->PreviousValue =  LastModifiedRamp->PreviousValue;
           Ramps_ResetRamp(LastModifiedRamp);  
        }
                
        LastModifiedRamp = pHandle;         
    }    
    
    static uint16_t Counter = 0; 
    
    switch(pHandle->RampType)
    {
       case LINEAR:
           ValueOut = Ramps_ApplyLinearRamp(pHandle,Input);
           break;
       case NO_RAMP:
       default:
           ValueOut = Input;
           break;
    }  
    
    // We need to slow down the ramp to ensure we have time between set points
    // but if we receive a request to cut the power, we don't wait
    
    if (Counter > RAMP_TIMESCALE_COUNTER || ValueOut == 0 || pHandle->RampType == NO_RAMP) 
    {                                                      
        Counter = 0; // If its time for a new setpoint reset the counter and dont overwrite the value
    }
    else
    {
        Counter ++;  // If it's not the time keep counting and overwrite with the previous value
        ValueOut = pHandle->PreviousValue;
    } 
    
    pHandle->PreviousValue = ValueOut; // Updated the last value
    
    return ValueOut; 
}

/**
  Function used to apply a linear ramp
*/
uint16_t Ramps_ApplyLinearRamp(Ramps_Handle_t * pHandle, uint16_t Input)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->RampType == LINEAR); 
   
    uint16_t ValOut = 0; 
     
    
    if (Input == 0) // If we need to cut the power don't check the next value of the ramp simply apply it
    {    
        ValOut = Input; 
    }
    // Do we need to apply the restriction  for accelerating ?
    else if (Input > (pHandle->PreviousValue + pHandle->LinearParameters.MaxDelta) && pHandle->RampDirection == ACCELERATION) 
    {
        ValOut = (uint16_t)round(pHandle->PreviousValue + pHandle->LinearParameters.MaxDelta);  // Apply the restricted value      
    }
    // Do we need to apply the restriction  for decelerating ?
    else if (Input < (pHandle->PreviousValue - pHandle->LinearParameters.MaxDelta) && pHandle->RampDirection == DECELERATION)
    {            
        if ((pHandle->PreviousValue - pHandle->LinearParameters.MaxDelta) < 0) // make sure we saturate at 0 power
        {
            ValOut = 0;  
        }    
        else
        {
            ValOut = (uint16_t)round(pHandle->PreviousValue - pHandle->LinearParameters.MaxDelta); // Apply the restricted value 
        }
    }
    else // If the value is within the max range then simply apply it
    {
        ValOut = Input;    
    }
    
    return ValOut;    
}

/**
  Function used reset a ramp
*/
void Ramps_ResetRamp(Ramps_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    switch(pHandle->RampType)
    {
       case LINEAR:
           pHandle->PreviousValue = 0;
           break;
       case NO_RAMP:
       default:
            // Do nothing
           break;
    } 
}