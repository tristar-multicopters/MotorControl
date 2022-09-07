/**
    * @file    foldback.c
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
    * @author  Ronak NemaDE, FTEX
    * @brief   This module implement a negative ramp to limit an input value based on another variable.
    *
*/
	
#include "foldback.h"

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

void Foldback_InitFoldback( Foldback_Handle_t * pHandle )
{
    pHandle->hMaxOutputLimitHigh = (int16_t) pHandle->hDefaultOutputLimitHigh;
    pHandle->hMaxOutputLimitLow = (int16_t) pHandle->hDefaultOutputLimitLow;
}

int16_t Foldback_GetMaxOutput(Foldback_Handle_t * pHandle, int16_t hValue)
{
	int16_t hMaxOutput = 0;
    int16_t hStartValue;
    uint32_t wAux;
    
    if (!pHandle->bIsInverted)
    {
        hStartValue = pHandle->hDecreasingEndValue - pHandle->hDecreasingRange;
    }
    else
    {
        hStartValue = pHandle->hDecreasingEndValue + pHandle->hDecreasingRange;
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

    int16_t hMaxOutput,hOutputVariable = 0;
    
	if (pHandle->bEnableFoldback)
	{
            if(pHandle->FoldbackConfig == TRIM)  // Test if foldback instance is used to trim the inputs.
            {
                hMaxOutput =  Foldback_GetMaxOutput(pHandle, hValue);
                if ( hInputVariable > hMaxOutput )  // If input is greater than maximum possible value 
                {
                    hOutputVariable = hMaxOutput; 
                }  
                else if ( hInputVariable < - hMaxOutput ) // If input is smaller than maximum possible value               
                { 
                    hOutputVariable = - hMaxOutput;
                } 
                else 
                { 
                    hOutputVariable = hInputVariable;} // set output as input
            }    
            else                    // Foldback instance is used to calculate dynamic thresholds.
            {
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
 * Check function definition
 **/
void Foldback_SetDecreasingRange(Foldback_Handle_t * pHandle, uint16_t hDecreasingRange)
{
    pHandle->hDecreasingRange = hDecreasingRange;
}

/**
 * Check function definition
 **/
void Foldback_SetDecreasingEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingEndValue)
{
    pHandle->hDecreasingEndValue = hDecreasingEndValue;
}

/**
 * Check function definition
 **/
void Foldback_EnableFoldback(Foldback_Handle_t * pHandle)
{
    pHandle->bEnableFoldback = true;
}

/**
 * Check function definition
 **/
void Foldback_DisableFoldback(Foldback_Handle_t * pHandle)
{
    pHandle->bEnableFoldback = false;
}

/**
 * Check function definition
 **/
void Foldback_UpdateMaxValue(Foldback_Handle_t * pHandle, int16_t hMaxValue)
{
    if(pHandle->FoldbackConfig == TRIM)
    {
        pHandle->hMaxOutputLimitHigh = hMaxValue; 
    }
    
}
