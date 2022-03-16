/**
  ******************************************************************************
  * @file    motor_selection.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles motor selection inputs
  *
	******************************************************************************
*/

#include "motor_selection.h"


static void M1Select_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
}

static void M2Select_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
}

/**
 * @brief Initializes torque distribution module
 *
 *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
 */
void MS_Init( MS_Handle_t * pHandle )
{
	nrfx_gpiote_in_config_t in_config = 
	{
		.sense = NRF_GPIOTE_POLARITY_TOGGLE,
		.pull = NRF_GPIO_PIN_PULLUP,
		.is_watcher = false,
		.hi_accuracy = true,
		.skip_gpio_setup = false,
	};
	nrfx_gpiote_in_init(pHandle->wM1SelectPinNumber, &in_config, M1Select_callback);
	nrfx_gpiote_in_init(pHandle->wM2SelectPinNumber, &in_config, M2Select_callback);
	
	//nrfx_gpiote_in_event_enable(pHandle->wM1SelectPinNumber, true);
	//nrfx_gpiote_in_event_enable(pHandle->wM2SelectPinNumber, true);
}

MotorSelection_t MS_CheckSelection(MS_Handle_t* pHandle)
{
	bool bM1Select = nrfx_gpiote_in_is_set(pHandle->wM1SelectPinNumber);
	bool bM2Select = nrfx_gpiote_in_is_set(pHandle->wM2SelectPinNumber);
	bool bBothSelect = bM1Select && bM2Select;
	
	if (bBothSelect)
	{
		pHandle->bMotorSelection = ALL_MOTOR_SELECTED;
	}
	else if (bM2Select)
	{
		pHandle->bMotorSelection = M2_SELECTED;
	}
	else
	{
		pHandle->bMotorSelection = M1_SELECTED;
	}
	
	return pHandle->bMotorSelection;
}

