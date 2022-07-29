/**
    * @file    foldback.c
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
    * @brief   This module implement a negative ramp to limit an input value based on another variable.
    *
*/
	
#include "foldback.h"

static int16_t Foldback_GetMaxOutput(Foldback_Handle_t * pHandle, int16_t hControlVariable);

/**
  * @brief  Function for computing maximum output value based on a control variable
  * @param  pHandle: handler of the current instance of the Foldback component
  * @param  hControlVariable: Variable that define the output limit
  * @retval Computed output limit
  */
static int16_t Foldback_GetMaxOutput(Foldback_Handle_t * pHandle, int16_t hControlVariable);


int16_t Foldback_GetMaxOutput(Foldback_Handle_t * pHandle, int16_t hValue)
{
	int16_t hTorqueMax = 0;
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

	if (pHandle->bEnableFoldback)
	{	
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
				wAux = pHandle->hDefaultMaxOutput * (pHandle->hDecreasingEndValue - hValue);
				wAux /= (uint32_t)(pHandle->hDecreasingEndValue - hStartValue);
				hTorqueMax = (int16_t)wAux;
			}
			else if (hValue <= hStartValue)
			{
				hTorqueMax = pHandle->hDefaultMaxOutput;
			}
			else
			{
				hTorqueMax = 0;
			}
		}
		else
		{
			// If hValue is in the foldback range
			if(hValue < hStartValue && hValue > pHandle->hDecreasingEndValue)
			{ 
				// Find the max torque value according to the foldback settings
				wAux = pHandle->hDefaultMaxOutput * (pHandle->hDecreasingEndValue - hValue);
				wAux /= (int32_t)(hStartValue - pHandle->hDecreasingEndValue);
				hTorqueMax = (int16_t)wAux;
			}
			else if (hValue >= hStartValue)
			{
				hTorqueMax = pHandle->hDefaultMaxOutput;
			}
			else
			{
				hTorqueMax = 0;
			}
		}
	}
	
	return hTorqueMax;
}


int16_t Foldback_ApplyFoldback(Foldback_Handle_t * pHandle, int16_t hInputVariable, int16_t hValue)
{
    int16_t hTorqueOut = 0;
    
	if (pHandle->bEnableFoldback)
	{
		int16_t hTorqueMax = Foldback_GetMaxOutput(pHandle, hValue);

		if (hInputVariable > hTorqueMax)
		{
			hTorqueOut = hTorqueMax;
		}
		else if (hInputVariable < -hTorqueMax)
		{
			hTorqueOut = -hTorqueMax;
		}
		else
		{
			hTorqueOut = hInputVariable;
		}
	}
	else
	{
		hTorqueOut = hInputVariable;
	}
	
	return hTorqueOut;
}


void Foldback_SetDecreasingRange(Foldback_Handle_t * pHandle, uint16_t hDecreasingRange)
{
    pHandle->hDecreasingRange = hDecreasingRange;
}


void Foldback_SetDecreasingEndValue(Foldback_Handle_t * pHandle, int16_t hDecreasingEndValue)
{
    pHandle->hDecreasingEndValue = hDecreasingEndValue;
}

void Foldback_EnableFoldback(Foldback_Handle_t * pHandle)
{
    pHandle->bEnableFoldback = true;
}

void Foldback_DisableFoldback(Foldback_Handle_t * pHandle)
{
    pHandle->bEnableFoldback = false;
}
