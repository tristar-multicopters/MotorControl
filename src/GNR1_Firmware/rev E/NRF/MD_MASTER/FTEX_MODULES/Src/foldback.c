/**
  ******************************************************************************
  * @file    foldback.c
	* @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
  * @brief   This module manages the vehicle and motor foldbacks 
  *
	******************************************************************************
	*/
	
#include "foldback.h"

/**@brief Function for calculating the maximum torque value from foldback curve.
	        The curve response is done using the following expressions
					1) hDefaultMaxTorque = hStartValue*m + b,
					2)    0	  		  = hEndValue*m + b
					So the curve formula is : f(x) = hDefaultMaxTorque*(hEndValue - hValue)/(hEndValue - hStartValue)
	 @p hValue : Input value (voltage, temperature, ...)
*/
int16_t FLDBK_CalcTorqueMax(FLDBK_Handle_t * pHandle, uint16_t hValue)
{
	int16_t hTorqueMax;
	
	// If hValue is in the foldback range
	if(hValue >= pHandle->hStartValue && hValue <= pHandle->hEndValue)
	{ 
		// Find the max torque value according to the foldback settings
		uint32_t wAux = pHandle->hDefaultMaxTorque * (pHandle->hEndValue - hValue);
		wAux /= (uint16_t)(pHandle->hEndValue - pHandle->hStartValue);
		hTorqueMax = (int16_t)wAux;
	}
	else if (hValue < pHandle->hStartValue)
	{
		hTorqueMax = pHandle->hDefaultMaxTorque;
	}
	else
	{
		hTorqueMax = 0;
	}
	
	return hTorqueMax;
}


int16_t FLDBK_ApplyTorqueLimitation(FLDBK_Handle_t * pHandle, int16_t hInitialTorque, uint16_t hValue)
{
	int16_t hTorqueOut;
	int16_t hTorqueMax = FLDBK_CalcTorqueMax(pHandle, hValue);
	
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
