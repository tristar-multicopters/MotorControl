/**
  ******************************************************************************
  * @file    brake.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles brake sensor management
  *
	******************************************************************************
*/

#include "brake.h"

static BRK_Handle_t * m_pBrake;

/* Functions ---------------------------------------------------- */

static void brake_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
}

/**
 * @brief Initializes brake sensor module
 */
void BRK_Init( BRK_Handle_t * pHandle )
{
	m_pBrake = pHandle;
	
	nrfx_gpiote_in_config_t in_config = 
	{
		.sense = NRF_GPIOTE_POLARITY_TOGGLE,
		.pull = NRF_GPIO_PIN_PULLUP,
		.is_watcher = false,
		.hi_accuracy = true,
		.skip_gpio_setup = false,
	};
	nrfx_gpiote_in_init(pHandle->wPinNumber, &in_config, brake_callback);
	//nrfx_gpiote_in_event_enable(pHandle->wPinNumber, true);
}

bool BRK_IsPressed( BRK_Handle_t * pHandle )
{
	bool bAux = nrfx_gpiote_in_is_set(pHandle->wPinNumber);
	pHandle->bIsPressed = bAux ^ pHandle-> bIsInvertedLogic;
	
	return pHandle->bIsPressed;
}

