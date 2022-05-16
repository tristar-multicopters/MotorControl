/*
  ******************************************************************************
  * @file    gnr_ra6t2_it.c
	* @author  FTEX inc
  * @brief   Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "mc_config.h"
#include "mc_tasks.h"

void CS_ADC_IRQHandler(adc_callback_args_t * p_args)
{
	if (p_args->event == ADC_EVENT_SCAN_COMPLETE)
	{
		TSK_HighFrequencyTask();
	}
	
}

void PWM_TIM_UP_IRQHandler(timer_callback_args_t * p_args)
{
	
}