/**
  * @file    Delay.c
  * @brief   This module handles delay that are based on counters
  *
  */

#include "delay.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
 *  Initializes Delay 
 */
void Delay_Init(Delay_Handle_t * pHandle, uint32_t aPulseTime, DelayUnits_t aPulseUnit)
{
    ASSERT(pHandle != NULL);
    
    if(aPulseTime && aPulseUnit != NULL)
    {
        pHandle->TimePerPulse = aPulseTime;
        pHandle->TimePerPulseUnits = aPulseUnit;
        pHandle->DelayInitialized = true;
    }
    else
    {
        pHandle->DelayInitialized = false;
    }
    
}

/**
 *  Update the Delay 
 */
bool Delay_Update(Delay_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if(pHandle->DelayCounter >= pHandle->DelayTimePulse) // Overflow detection 
    {
        Delay_Reset(pHandle);
        return true;          
    }       
    else
    {
        pHandle->DelayCounter ++;
        return false;
    }    
}

/**
 *  Reset the Delay
 */
void Delay_Reset(Delay_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->DelayCounter = 0;    
}

/**
 *  Set the time length of the delay
 */
void Delay_SetTime(Delay_Handle_t * pHandle, uint32_t aDelay, DelayUnits_t aDelayUnits)
{
    ASSERT(pHandle != NULL);
    
    ASSERT(pHandle->TimePerPulseUnits <= aDelayUnits); // If we want a milisecond delay but we receive pulses every second 
                                                      // it is not possible to make the Delay
    
    if(pHandle->TimePerPulseUnits == aDelayUnits && aDelay < pHandle->TimePerPulse) // If we want a delay every 6 ms but the pusle                                                                   
    {                                                                               // is set at 20 ms then its not possible
        ASSERT(false);
    }
    
    uint32_t ConversionRate = 1;    
    
    switch(pHandle->TimePerPulseUnits) // Depending on the time reference for the pulse
    {
        case MIC_SEC: // If the reference is in Microseconds
            if (aDelayUnits == MIL_SEC)  // If we need a delay in Milisecondes
            {
                ConversionRate = 1000;
            }
            else if (aDelayUnits == SEC) // If we need a delay in Secondes  
            {
                ConversionRate = 1000000; 
            }
          break;            
        case MIL_SEC: // If the reference is in Milliseconds
            if (aDelayUnits == SEC)    // If we need a delay in Secondes
            {
               ConversionRate = 1000;
            }
          break;  
        case SEC:    // If the reference is in Seconds then we also have a delay in seconds and no conversion is needed
          break;
            
    }
    
    if (((aDelay * ConversionRate)) % pHandle->TimePerPulse == 0) // Do we have a common divisior ?
    {
        pHandle->DelayTimePulse = ((aDelay * ConversionRate)/pHandle->TimePerPulse); 
    }
    else
    {
        pHandle->DelayTimePulse = ((aDelay * ConversionRate)/pHandle->TimePerPulse) + 1; // We would rather have a slightly longer delay then slightly shorter
    }
}