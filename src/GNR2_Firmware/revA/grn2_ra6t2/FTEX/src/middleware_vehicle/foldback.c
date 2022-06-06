/**
  * @file    foldback.c
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
  * @brief   This module manages the vehicle and motor foldbacks 
  *
	*/
	
#include "foldback.h"

/**@brief Function for calculating the maximum torque value from foldback curve.
	        The curve response is done using the following expressions
					1) hDefaultMaxTorque = hStartValue*m + b,
					2)    0	  		  = hEndValue*m + b
					So the curve formula is : f(x) = hDefaultMaxTorque*(hEndValue - hValue)/(hEndValue - hStartValue)
	 @p hValue : Input value (voltage, temperature, ...)
*/
int16_t FLDBK_CalcTorqueMax(FLDBK_Handle_t * pHandle, int16_t hValue)
{
	int16_t hTorqueMax = 0;
	
	if (pHandle->bEnableFoldback)
	{	
		if (pHandle->hStartValue > pHandle->hEndValue)
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
			if(hValue > pHandle->hStartValue && hValue < pHandle->hEndValue)
			{ 
				// Find the max torque value according to the foldback settings
				uint32_t wAux = pHandle->hDefaultMaxTorque * (pHandle->hEndValue - hValue);
				wAux /= (int16_t)(pHandle->hEndValue - pHandle->hStartValue);
				hTorqueMax = (int16_t)wAux;
			}
			else if (hValue <= pHandle->hStartValue)
			{
				hTorqueMax = pHandle->hDefaultMaxTorque;
			}
			else
			{
				hTorqueMax = 0;
			}
		}
		else
		{
			// If hValue is in the foldback range
			if(hValue < pHandle->hStartValue && hValue > pHandle->hEndValue)
			{ 
				// Find the max torque value according to the foldback settings
				uint32_t wAux = pHandle->hDefaultMaxTorque * (pHandle->hEndValue - hValue);
				wAux /= (int16_t)(pHandle->hStartValue - pHandle->hEndValue);
				hTorqueMax = (int16_t)wAux;
			}
			else if (hValue >= pHandle->hStartValue)
			{
				hTorqueMax = pHandle->hDefaultMaxTorque;
			}
			else
			{
				hTorqueMax = 0;
			}
		}
	}
	
	return hTorqueMax;
}


int16_t FLDBK_ApplyTorqueLimitation(FLDBK_Handle_t * pHandle, int16_t hInitialTorque, int16_t hValue)
{
		int16_t hTorqueMax = FLDBK_CalcTorqueMax(pHandle, hValue);
		int16_t hTorqueOut = 0;
	
	if (pHandle->bEnableFoldback)
	{
		if (hInitialTorque > hTorqueMax)
		{
			hTorqueOut = hTorqueMax;
		}
		else if (hInitialTorque < -hTorqueMax)
		{
			hTorqueOut = -hTorqueMax;
		}
		else
		{
			hTorqueOut = hInitialTorque;
		}
	}
	else
	{
		hTorqueOut = hInitialTorque;
	}
	
	return hTorqueOut;
}

/* Function for setting the start speed limitation value */
void FLDBK_SetStartValue (FLDBK_Handle_t * pHandle, uint16_t hStartValue)
{
		pHandle->hStartValue = hStartValue;
}

/* Function for setting the end limitation speed value */
void FLDBK_SetEndValue (FLDBK_Handle_t * pHandle, uint16_t hStartValue)
{
	int16_t hInterval, hEndval;
	/* Add the Interval Value to the Start Value */
	hInterval = pHandle->hIntervalValue;
	hEndval = hInterval + hStartValue;
	pHandle->hEndValue = hEndval;
}



