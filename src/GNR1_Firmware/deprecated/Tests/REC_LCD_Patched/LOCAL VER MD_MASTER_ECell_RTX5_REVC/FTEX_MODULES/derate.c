/**
  ******************************************************************************
  * @file    derate.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles derating of the device, i.e. when temperature exceed safe area.
  *
	******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "derate.h"


/* Functions ---------------------------------------------------- */


/**
 * @brief Initializes derate management
 *
 *  @p pHandle : Pointer on Handle structure of derate component
 */
__weak void DRT_Init( DRT_Handle_t * pHandle )
{	
	pHandle->bDeratingON = false;
	pHandle->hTref = 0;
}

/**
  * @brief  Returns torque value limited by derating
  *
  * @p pHandle : Pointer on Handle structure of derate component
	* @p torque : Initial torque value 
  */
int16_t DRT_CalcDeratedTorque( DRT_Handle_t * pHandle, int16_t torque, int16_t temperature)
{
	int32_t wAux;
	int16_t hTorque;
	uint16_t hLevel;
	
	bool isTorquePositive = (torque >= 0) ? true : false;
	pHandle->bDeratingON = (temperature >= pHandle->hTempThreshold) ? true : false;
	
	if ( pHandle->bDeratingON )
	{
		wAux = pHandle->hSlope * (temperature - pHandle->hTempThreshold);
		if (wAux >= INT16_MAX)
		{
			wAux = INT16_MAX;
		}
		if (wAux <= INT16_MIN)
		{
			wAux = INT16_MIN;
		}
		if ( isTorquePositive )
		{
			hTorque = torque - wAux;
			if (hTorque <= 0)
			{
				hTorque = 0;
			}
		}
		else
		{
			hTorque = torque + wAux;
			if (hTorque >= 0)
			{
				hTorque = 0;
			}
		}
	}
	else
	{
		hTorque = torque;
	}
	
	pHandle->hTref = hTorque;
	return hTorque;
}

