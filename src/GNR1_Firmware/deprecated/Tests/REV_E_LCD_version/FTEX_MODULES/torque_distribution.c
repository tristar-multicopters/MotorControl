/**
  ******************************************************************************
  * @file    torque_distribution.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles torque distribution between motors
  *
	******************************************************************************
*/

#include "torque_distribution.h"


/* Functions ---------------------------------------------------- */


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
void TD_Init( TD_Handle_t * pHandle )
{
	pHandle->aTorque[M1] = 0;
	pHandle->aTorque[M2] = 0;
	
	nrfx_gpiote_in_config_t in_config = 
	{
		.sense = NRF_GPIOTE_POLARITY_TOGGLE,
		.pull = NRF_GPIO_PIN_PULLDOWN,
		.is_watcher = false,
		.hi_accuracy = true,
		.skip_gpio_setup = false,
	};
	nrfx_gpiote_in_init(pHandle->wM1SelectPinNumber, &in_config, M1Select_callback);
	nrfx_gpiote_in_init(pHandle->wM2SelectPinNumber, &in_config, M2Select_callback);
	
	//nrfx_gpiote_in_event_enable(pHandle->wM1SelectPinNumber, true);
	//nrfx_gpiote_in_event_enable(pHandle->wM2SelectPinNumber, true);
}

/**
  * @brief Performs the torque distribution algorithm
  *
  *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
  */
void TD_DistributeTorque( TD_Handle_t * pHandle, int16_t hIqref)
{
	pHandle->aTorque[M1] = 0;
	pHandle->aTorque[M2] = 0;
	
	#if MOTOR_SELECTOR_PIN_ENABLE
	bool bM1Select = nrfx_gpiote_in_is_set(pHandle->wM1SelectPinNumber);
	bool bM2Select = nrfx_gpiote_in_is_set(pHandle->wM2SelectPinNumber);
	
	pHandle->bMode = !bM1Select && !bM2Select;
	if (pHandle->bMode == DUAL_MOTOR)
		pHandle->bMainMotorSelection = M1;
	else
		pHandle->bMainMotorSelection = bM2Select;
	#endif
	
	if(pHandle->bMode == SINGLE_MOTOR)
	{
		pHandle->aTorque[pHandle->bMainMotorSelection] = hIqref;
	}
	if(pHandle->bMode == DUAL_MOTOR)
	{
		pHandle->aTorque[M1] = hIqref;
		pHandle->aTorque[M2] = hIqref;
	}
}

int16_t TD_GetTorqueM1(TD_Handle_t * pHandle)
{
	return pHandle->aTorque[M1];
}

int16_t TD_GetTorqueM2(TD_Handle_t * pHandle)
{
	return pHandle->aTorque[M2];
}

int16_t TD_GetTorqueMainMotor(TD_Handle_t * pHandle)
{
	return pHandle->aTorque[pHandle->bMainMotorSelection];
}

