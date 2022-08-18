/**
  * @file    power_enable.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles power enable pin
  *
*/

#include "power_enable.h"
#include "vc_config.h"

/**
 * @brief Initializes brake sensor module
 */
void PWREN_Init(PWREN_Handle_t * pHandle)
{	
	ASSERT(pHandle != NULL);
}

bool PWREN_IsPowerEnabled(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
	if (pHandle->bUsePowerLock)
	{
		bool bAux = true; //TODO: check gpio input, for now just mock
		pHandle->bIsPowerEnabled = bAux ^ pHandle-> bIsInvertedLogic;
		
		return pHandle->bIsPowerEnabled;
	}
	else
		return true;
}
