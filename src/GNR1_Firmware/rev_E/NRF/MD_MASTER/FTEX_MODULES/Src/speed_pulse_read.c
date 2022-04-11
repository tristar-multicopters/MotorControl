/**
  ******************************************************************************
  * @file           : speed_pulse_read.c
  * @brief          : This module handles speed pulse read frequency initilization
	*										and configuration
	* @author  				: Jabrane Chakroun
  ******************************************************************************
  * Hardware used:
  *  	P0.0 -----
	*		Timer ----
	*			=> Defined in vc_config.c
  *  
  * ------------------------------
  * Based on software:
  *  	PPI SDK from Nordic
  *   GPIOTE SDK from Nordic
	*		TIMER SDK from Nordic
  * ------------------------------
  * Version number:
  *  0.0.1
  * ------------------------------
  * Last modified:
  *  10.03.2022 
  * by 
  *  Jabrane Chakroun
  * ------------------------------
  *
  * This software is the sole property of FTEX Inc.
  *
  * COPYRIGHT (c) 2022 BY FTEX Inc. ALL RIGHTS RESERVED. NO PART OF
  * THIS PROGRAM OR PUBLICATION MAY BE REPRODUCED, TRANSMITTED, TRANSCRIBED, 
  * STORED IN A RETRIEVAL SYSTEM, OR TRANSLATED INTO ANY LANGUAGE OR COMPUTER 
  * LANGUAGE IN ANY FORM OR BY ANY MEANS, ELECTRONIC, MECHANICAL, MAGNETIC, 
  * OPTICAL, CHEMICAL, MANUAL, OR OTHERWISE, WITHOUT THE PRIOR WRITTEN 
  * PERMISSION OF FTEX Inc.
  *
  ******************************************************************************
  */
	
/******************************************************************************* Includes ********************************************************************************/
#include "speed_pulse_read.h"

/**************************************************************************** Private definitions ************************************************************************/
static SPR_Handle_t* p_SPR_Handle;	
/* Direction flags */
bool sin_flag;
bool cos_flag;
/* Speed general variables */
uint16_t pSpeed;
uint16_t wSpeed;
/****************************************************************** Public Hardware dependent functions ******************************************************************/

/**
	* @brief  Module initialization, to be called for the 
	*					SPR initialization process
	* @param  Handle pointer
	* @retval None
	*/
void SPR_Init(SPR_Handle_t * pHandle)
{
	GPIOTE_Capture_Init(pHandle); 
	GPIO_Init(pHandle);
}
/**
	* @brief  Module initialization, to be called for the 
	*					Wheel SPR initialization process
	* @param  Handle pointer
	* @retval None
	*/
void SPWR_Init(SPR_Handle_t * pHandle)
{
	GPIOTE_Wheel_Capture_Init(pHandle);
}


/**
	* @brief  GPIO interrupt initialization
	* @param  Handle pointer
	* @retval None
	*/
void GPIO_Init(SPR_Handle_t* pHandle)
{
	
	p_SPR_Handle = pHandle;

	/* Already Declared in the start of the General Initialization */
	//err_code = nrf_drv_gpiote_init();

	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	//in_config.pull = NRF_GPIO_PIN_PULLDOWN;

	nrf_drv_gpiote_in_init(pHandle->pSinSpeed_Pulse_pin, &in_config, GPIO_Pin_handler);

	nrf_drv_gpiote_in_init(pHandle->pCosSpeed_Pulse_pin, &in_config, GPIO_Pin_handler);

	
	nrfx_err_t nrfx_gpiote_in_init(nrfx_gpiote_pin_t               pin,
                               nrfx_gpiote_in_config_t const * p_config,
                               nrfx_gpiote_evt_handler_t       evt_handler);
	
	
	
	
	nrf_drv_gpiote_in_event_enable(pHandle->pSinSpeed_Pulse_pin, true);
	nrf_drv_gpiote_in_event_enable(pHandle->pCosSpeed_Pulse_pin, true);
}

/**
	* @brief  GPIOTE Peadl pulse Capture initialization
	* @param  Handle pointer
	* @retval None
	*/
uint8_t GPIOTE_Capture_Init(SPR_Handle_t* pHandle)  
{
	
	p_SPR_Handle = pHandle;
	uint8_t err_code;
	
	/* definition of ppi channel for the first conversion */
	static nrf_ppi_channel_t ppi_ch_gpiote_capture;
	static nrf_ppi_channel_t ppi_ch_gpiote_restart;
	
	/* Optionally, enable pullup or pulldown on the input pin */
	nrf_gpio_cfg_input(pHandle->pSinSpeed_Pulse_pin, NRF_GPIO_PIN_PULLDOWN);

	/* Allocate two PPI channels to the GPIOTE */
	nrfx_ppi_channel_alloc(&ppi_ch_gpiote_capture);
	nrfx_ppi_channel_alloc(&ppi_ch_gpiote_restart);

	/* Capture Timer Configuration */
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_cfg.frequency = pHandle->bTimer_Prescaler;
	timer_cfg.bit_width = pHandle->bTimer_Width;
	
	/* Initialize Capture timer */ 
	err_code = nrf_drv_timer_init(pHandle->pTimerInstance, &timer_cfg, 0);
	err_code = nrf_drv_timer_init(pHandle->wTimerInstance, &timer_cfg, 0);

	//APP_ERROR_CHECK(err_code);

	// The GPIOTE driver doesn't support two GPIOTE channels on the same pin, so direct register access is necessary
	NRF_GPIOTE->CONFIG[pHandle->bCaptureChannel] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
																					GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos |
																					pHandle->pSinSpeed_Pulse_pin << GPIOTE_CONFIG_PSEL_Pos;
	NRF_GPIOTE->CONFIG[pHandle->bRestartChannel] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
																					GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos |
																					pHandle->pSinSpeed_Pulse_pin << GPIOTE_CONFIG_PSEL_Pos;


	// Assign a PPI channel to capture the current timer state and store it in CC register 0
	nrfx_ppi_channel_assign(ppi_ch_gpiote_capture, 
													(uint32_t)&NRF_GPIOTE->EVENTS_IN[pHandle->bCaptureChannel], 
													nrf_drv_timer_capture_task_address_get(pHandle->pTimerInstance, 0));
													
											
	
	// Assign a second PPI channel to restart the timer when a new pulse is detected
	nrfx_ppi_channel_assign(ppi_ch_gpiote_restart, 
													(uint32_t)&NRF_GPIOTE->EVENTS_IN[pHandle->bRestartChannel], 
													nrf_drv_timer_task_address_get(pHandle->pTimerInstance, NRF_TIMER_TASK_CLEAR));
	
	// Enable both PPI channels                
	nrfx_ppi_channel_enable(ppi_ch_gpiote_capture);
	nrfx_ppi_channel_enable(ppi_ch_gpiote_restart);

	// Make sure the GPIOTE capture in event is cleared. This will be  used to detect if a capture has occured. 
	NRF_GPIOTE->EVENTS_IN[pHandle->bCaptureChannel] = 0;

	// Start the timer
	nrfx_timer_resume(pHandle->pTimerInstance);		
	
	return err_code;
}


/**
	* @brief  GPIOTE Wheel pulse Capture initialization
	* @param  Handle pointer
	* @retval None
	*/
uint8_t GPIOTE_Wheel_Capture_Init(SPR_Handle_t* pHandle)  
{
	
	p_SPR_Handle = pHandle;
	uint8_t err_code;
	
	/* definition of ppi channel for the first conversion */
	static nrf_ppi_channel_t ppi_ch_gpiote_capture_2;
	static nrf_ppi_channel_t ppi_ch_gpiote_restart_2;
	
	/* Optionally, enable pullup or pulldown on the input pin */
	nrf_gpio_cfg_input(pHandle->pWheelSpeed_Pulse_pin, NRF_GPIO_PIN_PULLDOWN);

	/* Allocate two PPI channels to the GPIOTE */
	nrfx_ppi_channel_alloc(&ppi_ch_gpiote_capture_2);
	nrfx_ppi_channel_alloc(&ppi_ch_gpiote_restart_2);

	/* Capture Timer Configuration */
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_cfg.frequency = pHandle->bTimer_Prescaler;
	timer_cfg.bit_width = pHandle->bTimer_Width;
	
	/* Initialize Capture timer */ 
	err_code = nrf_drv_timer_init(pHandle->wTimerInstance, &timer_cfg, 0);
	//APP_ERROR_CHECK(err_code);

	// The GPIOTE driver doesn't support two GPIOTE channels on the same pin, so direct register access is necessary
	NRF_GPIOTE->CONFIG[pHandle->WCaptureChannel] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
																					GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos |
																					pHandle->pWheelSpeed_Pulse_pin << GPIOTE_CONFIG_PSEL_Pos;
	NRF_GPIOTE->CONFIG[pHandle->WRestartChannel] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
																					GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos |
																					pHandle->pWheelSpeed_Pulse_pin << GPIOTE_CONFIG_PSEL_Pos;


	// Assign a PPI channel to capture the current timer state and store it in CC register 0
	nrfx_ppi_channel_assign(ppi_ch_gpiote_capture_2, 
													(uint32_t)&NRF_GPIOTE->EVENTS_IN[pHandle->WCaptureChannel], 
													nrf_drv_timer_capture_task_address_get(pHandle->wTimerInstance, 0));
													
											
	
	// Assign a second PPI channel to restart the timer when a new pulse is detected
	nrfx_ppi_channel_assign(ppi_ch_gpiote_restart_2, 
													(uint32_t)&NRF_GPIOTE->EVENTS_IN[pHandle->WRestartChannel], 
													nrf_drv_timer_task_address_get(pHandle->pTimerInstance, NRF_TIMER_TASK_CLEAR));
	
	// Enable both PPI channels                
	nrfx_ppi_channel_enable(ppi_ch_gpiote_capture_2);
	nrfx_ppi_channel_enable(ppi_ch_gpiote_restart_2);

	// Make sure the GPIOTE capture in event is cleared. This will be  used to detect if a capture has occured. 
	NRF_GPIOTE->EVENTS_IN[pHandle->WCaptureChannel] = 0;

	// Start the timer
	nrfx_timer_resume(pHandle->wTimerInstance);
		
	return err_code;
}

/**
	* @brief  GPIOTE Wheel Capture initialization
	* @param  Handle pointer
	* @retval uint32_t periode value in us
	*/

uint16_t Pedal_capture_get_value(SPR_Handle_t* sHandle)
{
	p_SPR_Handle = sHandle;
	// Make sure the capture event occured before checking the capture register
	if((NRF_GPIOTE->EVENTS_IN[sHandle->bCaptureChannel] != 0)||(NRF_GPIOTE->EVENTS_IN[sHandle->bRestartChannel] != 0))
	{		// Clear the capture event
			NRF_GPIOTE->EVENTS_IN[sHandle->bCaptureChannel] = 0;
			NRF_GPIOTE->EVENTS_IN[sHandle->bRestartChannel] = 0;
			// Return the stored capture value in the timer
			sHandle->sPread =  nrf_drv_timer_capture_get(sHandle->pTimerInstance, (nrf_timer_cc_channel_t)0);
	}
	else
	{		// In case no capture occured, return 0
			sHandle->sPread = 0;
	}
	return (sHandle->sPread);
}

/**
	* @brief  GPIOTE Wheel Capture initialization
	* @param  Handle pointer
	* @retval uint32_t periode value in us
	*/
uint16_t Wheel_capture_get_value(SPR_Handle_t* sHandle)
{
	p_SPR_Handle = sHandle;
	// Make sure the capture event occured before checking the capture register
	if(NRF_GPIOTE->EVENTS_IN[sHandle->WCaptureChannel] != 0)
	{		// Clear the capture event
			NRF_GPIOTE->EVENTS_IN[sHandle->WCaptureChannel] = 0;	
			// Return the stored capture value in the timer
			sHandle->wPread =  nrf_drv_timer_capture_get(sHandle->wTimerInstance, (nrf_timer_cc_channel_t)0);		
	}
	else
	{		// In case no capture occured, return 0
			sHandle->wPread = 0;
	}
		return (sHandle->wPread);
}

/**
	* @brief  GPIO interrupt callaback
	* @param  Handle pointer
	* @retval direction decision 
	*/
void GPIO_Pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)//, SPR_Handle_t * sHandle)
{
	if(pin == p_SPR_Handle->pSinSpeed_Pulse_pin)
	{
       sin_flag = nrfx_gpiote_in_is_set(p_SPR_Handle->pSinSpeed_Pulse_pin);
       cos_flag = nrfx_gpiote_in_is_set(p_SPR_Handle->pCosSpeed_Pulse_pin);

		if (sin_flag && (!cos_flag))
        {
					p_SPR_Handle->Direction_result = Forward;
        }
		else if (cos_flag && (!sin_flag))
        {
					p_SPR_Handle->Direction_result = Error_Direction;
        }
		else 
        {
					p_SPR_Handle->Direction_result = Reverse;
        }
	}
}

/**
	* @brief  Get direction function
	* @param  Handle pointer
	* @retval direction decision 
	*/
uint8_t Get_Drive_Direction (SPR_Handle_t* pHandle)
{
	 return ( p_SPR_Handle->Direction_result );
}

/**
	* @brief  Pedal speed value calculation and filtering
	* @param  Handle pointer
	* @retval None
	*/
uint16_t Pspeed_CalcAvValue( SPR_Handle_t * pHandle )
{  
	uint32_t sPtemp;
	uint16_t sBandwidth;

	sBandwidth = pHandle->sParam.sLowPassFilterBW1;
	pSpeed = Pedal_capture_get_value(pHandle);

	if ( pSpeed != 0xFFFFu )
	{
		sPtemp =  ( uint32_t )( sBandwidth - 1u );
		sPtemp *= ( uint32_t ) ( pHandle->sPAvSpeed );
		sPtemp += pSpeed;
		sPtemp /= ( uint32_t )( sBandwidth );

		pHandle->sPAvSpeed = ( uint16_t ) sPtemp;
	}
	return pHandle->sPAvSpeed;
}


/**
	* @brief  Wheel speed value calculation and filtering
	* @param  Handle pointer
	* @retval None
	*/
uint16_t Wspeed_CalcAvValue( SPR_Handle_t * pHandle )
{
  uint32_t sWtemp;
	uint16_t sBandwidth;

	sBandwidth = pHandle->sParam.sLowPassFilterBW1;
	wSpeed = Wheel_capture_get_value(pHandle);

	if ( wSpeed != 0xFFFFu )
	{
		sWtemp =  ( uint32_t )( sBandwidth - 1u );
		sWtemp *= ( uint32_t ) ( pHandle->sWAvSpeed );
		sWtemp += wSpeed;
		sWtemp /= ( uint32_t )( sBandwidth );

		pHandle->sWAvSpeed = ( uint16_t ) sWtemp;
	}  
	return pHandle->sWAvSpeed;
}

/****************************************************** (C) COPYRIGHT FTEX Inc. *****END OF FILE**** ************************************************/

