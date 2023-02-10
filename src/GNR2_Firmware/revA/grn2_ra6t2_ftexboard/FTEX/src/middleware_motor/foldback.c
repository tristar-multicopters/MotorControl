/**
    * @file    foldback.c
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
    * @author  Ronak NemaDE, FTEX
    * @brief   This module implement a negative ramp to limit an input value based on another variable.
    *
*/
	
#include "foldback.h"
#include "ASSERT_FTEX.h"
/**
  * @brief  Function for computing maximum output value based on a control variable
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hControlVariable: Variable that define the output limit
  * @retval Computed output limit
  */
static int16_t Foldback_GetMaxOutput(Foldback_Handle_t * pHandle, int16_t hControlVariable);

/**
  * Refer to function definition
  **/

void Foldback_Init( Foldback_Handle_t * pHandle )
{
    ASSERT(pHandle != NULL);
    pHandle->hMaxOutputLimitHigh = (int16_t) pHandle->hDefaultOutputLimitHigh;
    pHandle->hMaxOutputLimitLow = (int16_t) pHandle->hDefaultOutputLimitLow;
}

int16_t Foldback_GetMaxOutput(Foldback_Handle_t * pHandle, int16_t hValue)
{
    ASSERT(pHandle != NULL); 
	int16_t hMaxOutput = 0;
    int16_t hStartValue;
    uint32_t wAux;
    
    if (!pHandle->bIsInverted)
    {
        hStartValue = pHandle->hDecreasingEndValue - (int16_t) pHandle->hDecreasingRange;
    }
    else
    {
        hStartValue = pHandle->hDecreasingEndValue + (int16_t) pHandle->hDecreasingRange;
    }

    if (hStartValue > pHandle->hDecreasingEndValue)
    {
        pHandle->bIsInverted = true;
    }
    else
    {
        pHandle->bIsInverted = false;
    }
		
    if (!pHandle->bIsInverted)
    {
        // If hValue is in the foldback range
        if(hValue > hStartValue && hValue < pHandle->hDecreasingEndValue)
        { 
            // Find the max torque value according to the foldback settings
            wAux = (uint32_t)( ( pHandle->hMaxOutputLimitHigh - pHandle->hMaxOutputLimitLow )*( pHandle->hDecreasingEndValue - hValue ) );
            wAux /= (uint32_t)(pHandle->hDecreasingEndValue - hStartValue);
            wAux += (uint32_t) pHandle->hMaxOutputLimitLow;
            hMaxOutput = (int16_t)wAux;
        }
        else if (hValue <= hStartValue)
        {
            hMaxOutput = pHandle->hMaxOutputLimitHigh;    // Pass higher threshold if input is lower than control range
        }
        else
        {
            hMaxOutput = pHandle->hMaxOutputLimitLow;     // Pass lower threshold if input is higher than control range
        }
    }
    else
    {
        // If hValue is in the foldback range
        if(hValue < hStartValue && hValue > pHandle->hDecreasingEndValue)
        { 
            // Find the max torque value according to the foldback settings
            wAux = (uint32_t)( ( pHandle->hMaxOutputLimitHigh - pHandle->hDefaultOutputLimitLow )*( hValue - pHandle->hDecreasingEndValue ) );                
            wAux /= (uint32_t)(hStartValue - pHandle->hDecreasingEndValue);
            wAux += (uint32_t) pHandle->hMaxOutputLimitLow;
            hMaxOutput = (int16_t)wAux;
        }
        else if (hValue >= hStartValue)
        {
            hMaxOutput =  pHandle->hMaxOutputLimitHigh;       // Pass higher threshold if input is lower than control range
        }
        else
        {
            hMaxOutput =  pHandle->hMaxOutputLimitLow;        // Pass lower threshold if input is higher than control range
        }
    }
	
	return hMaxOutput;
}


int16_t Foldback_ApplyFoldback(Foldback_Handle_t * pHandle, int16_t hInputVariable, int16_t hValue)
{
    ASSERT(pHandle != NULL); 
    int16_t hMaxOutput,hOutputVariable = 0;
    
	if (pHandle->bEnableFoldback)
	{
        if(pHandle->FoldbackConfig == TRIM) // Test if foldback instance is used to trim the inputs.
        {
            hMaxOutput =  Foldback_GetMaxOutput(pHandle, hValue);
            if (hInputVariable > hMaxOutput) // If input is greater than maximum possible value 
            {
                hOutputVariable = hMaxOutput; 
            }
            else if (hInputVariable < - hMaxOutput) // If input is smaller than maximum possible value               
            { 
                hOutputVariable = - hMaxOutput;
            }
            else 
            { 
                hOutputVariable = hInputVariable;} // set output as input
            }    
            else
            {
                // Foldback instance is used to calculate dynamic thresholds.
                hOutputVariable = Foldback_GetMaxOutput(pHandle, hValue); 
            }
        }
        else
        {
            hOutputVariable = hInputVariable;
        }
	return hOutputVariable;
}

/**
 * Function for setting the start speed limitation value
 **/
void Foldback_SetDecreasingRange(Foldback_Handle_t * pHandle, uint16_t hDecreasingRange)
{
    ASSERT(pHandle != NULL); 
    pHandle->hDecreasingRange = hDecreasingRange;
}

/**
 * Function for setting the end limitation speed value
 **/
void Foldback_SetDecreasingEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingEndValue)
{
    ASSERT(pHandle != NULL); 
    pHandle->hDecreasingEndValue = hDecreasingEndValue;
}

/**
 * Function for setting the end limitation speed value
 **/
void Foldback_SetDecreasingRangeEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingRange)
{
    ASSERT(pHandle != NULL); 
    int16_t hInterval, hEndval;
    /* Add the Interval Value to the Start Value */
	hInterval = pHandle->hDecreasingInterval;
    hEndval = hInterval + hDecreasingRange;
    pHandle->hDecreasingEndValue = hEndval;
}


/**
 * Check function definition
 **/
void Foldback_EnableFoldback(Foldback_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL); 
    pHandle->bEnableFoldback = true;
}

/**
 * Check function definition
 **/
void Foldback_DisableFoldback(Foldback_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL); 
    pHandle->bEnableFoldback = false;
}

/**
 * Check function definition
 **/
void Foldback_UpdateMaxValue(Foldback_Handle_t * pHandle, int16_t hMaxValue)
{
    ASSERT(pHandle != NULL); 
    if(pHandle->FoldbackConfig == TRIM)
    {
        pHandle->hMaxOutputLimitHigh = hMaxValue; 
    }
    
}

/**
 * Apply a low pass filter on the torque after for smooth 
 * accelearation with Pedal Assist
 **/

int16_t Foldback_ApplySlowStart(Foldback_Handle_t * pHandle, int16_t hTorque)
{
    ASSERT(pHandle != NULL); 
    static uint32_t wTimeCounter;
    static int16_t hAverageTorque;
           int32_t wTemp = 0;
           int16_t hTorqueOut;

    hTorqueOut = hTorque;
    uint16_t hBandwidth;

    if(pHandle->bRefreshSlowStart)  
    { // Used to reset the counter, the average torque and the 
      // slow start refresh for PAS speed limitation 
       wTimeCounter = 0; 
       hAverageTorque = 0; 
       pHandle->bRefreshSlowStart = false; 
    }    

    if(pHandle->bEnableSlowStart) //Check if a slow start was requested or is in progress
    {    
        if(abs(hTorque) > abs(hAverageTorque))
        {
            hBandwidth = pHandle->hSlowStartBandwidth;
        }
        else
        {
            hBandwidth = pHandle->hSlowStopBandwidth;
        }
            
            wTemp =  (hBandwidth - 1u); //Apply a low pass filter to the torque
            wTemp *= hAverageTorque;
            wTemp += hTorque;
            wTemp /= hBandwidth;
            // Add a foladback filtering check for value under 1
            if (wTemp<MINIMUMVAL)
            {
                wTemp = MINIMUMVAL;
            }

            hAverageTorque =  (int16_t) wTemp ;
            hTorqueOut = hAverageTorque;            

        if (wTimeCounter > pHandle->wSlowStartTimeout)          // Timeout condition is there to make sure we cant get stuck in a slow start
        {
            wTimeCounter = 0;
            hAverageTorque = 0; 
            pHandle->bEnableSlowStart = false;        
        }              
    }
    else
    {
      wTimeCounter = 0;
      hAverageTorque = 0;        
    }        

    return hTorqueOut;
}

/**
 * Used to start or refresh a slow start
**/
void Foldback_EnableSlowStart(Foldback_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);     
    pHandle->bRefreshSlowStart = true;
    pHandle->bEnableSlowStart = true;   
} 
