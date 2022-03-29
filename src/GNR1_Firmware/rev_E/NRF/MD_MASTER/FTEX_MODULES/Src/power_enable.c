/**
  ******************************************************************************
  * @file    power_enable.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles power enable pin
  *
	******************************************************************************
*/

#include "power_enable.h"
#include "vc_config.h"

extern uint32_t BafTrashCounter; 

extern MD_Comm_Handle_t MDComm;

/* Functions ---------------------------------------------------- */

static void pwren_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//BafTrashCounter = 0;
}

/**
 * @brief Initializes brake sensor module
 */
void PWREN_Init( PWREN_Handle_t * pHandle )
{	
	if (pHandle->bUsePowerLock)
	{
		nrfx_gpiote_in_config_t in_config = 
		{
			.sense = NRF_GPIOTE_POLARITY_TOGGLE,
			.pull = NRF_GPIO_PIN_PULLDOWN,
			.is_watcher = false,
			.hi_accuracy = true,
			.skip_gpio_setup = false,
		};
		nrfx_gpiote_in_init(pHandle->wPinNumber, &in_config, pwren_callback);
		nrfx_gpiote_in_event_enable(pHandle->wPinNumber, true);
	}
}

bool PWREN_IsPowerEnabled( PWREN_Handle_t * pHandle )
{
	if (pHandle->bUsePowerLock)
	{
		bool bAux = nrfx_gpiote_in_is_set(pHandle->wPinNumber);
		pHandle->bIsPowerEnabled = bAux ^ pHandle-> bIsInvertedLogic;
		
		return pHandle->bIsPowerEnabled;
	}
	else
		return true;
}
