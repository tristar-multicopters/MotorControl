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

/**
	* @brief  Apply a low pass filter on the torque after the break is
    *         used or after a fault to ensure we wont hit over current
    *         from an accelaration overshoot  
	* @param  Foldback handle and torque value
	* @retval Torque after passing it through the filter
	*/
int16_t FLDBK_ApplySlowStart(FLDBK_Handle_t * pHandle, int16_t hTorque)
{
    static uint32_t wTimeCounter;
    static  int16_t hAverageTorque;
            int32_t wTemp = 0;
            int16_t hTorqueOut;
    
    hTorqueOut = hTorque;
    
    if(pHandle->bRefreshSlowStart) //Used to reset the counter and the
    {
       wTimeCounter = 0; 
       hAverageTorque = 0; 
       pHandle->bRefreshSlowStart = false; 
    }    
    
    if (pHandle->bEnableSlowStart) //Check if a slow start was requested or is in progress
    {    
           
        if(abs(hTorque) >= abs(hAverageTorque)) //Apply the filter only on acceleration
        {
            wTimeCounter ++; 
            
            wTemp =  (pHandle->hSlowStartBandwidth - 1u); //Apply a low pass filter to the torque
            wTemp *= hAverageTorque;
            wTemp += hTorque;
            wTemp /= pHandle->hSlowStartBandwidth ;

            hAverageTorque =  wTemp;
            hTorqueOut = hAverageTorque;            
        }  

        if (abs(hAverageTorque) > (abs(hTorque) - (abs(hTorque)/40)) && abs(hTorque) > abs(hAverageTorque)) // First condition to exit the slow start is that the average torque 
        {                                                                                                   // is at least 97.5% of the value of the requested torque
            wTimeCounter = 0; 
            hAverageTorque = 0;
            pHandle->bEnableSlowStart = false;            
        }    
        else if (wTimeCounter > pHandle->wSlowStartTimeout)          // Timeout condition is there to make sure we cant get stuck in a slow start
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
	* @brief  Used to start or refresh a slow start
	* @param  Foldback handle 
	* @retval Nothing
	*/
void FLDBK_EnableSlowStart(FLDBK_Handle_t * pHandle)
{        
    pHandle->bRefreshSlowStart = true;
    pHandle->bEnableSlowStart = true;   
}    
