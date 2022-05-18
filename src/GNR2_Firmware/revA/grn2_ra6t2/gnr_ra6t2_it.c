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
#include "bsp_pin_cfg.h"

void CS_ADC_IRQHandler(adc_callback_args_t * p_args)
{
	if (p_args->event == ADC_EVENT_SCAN_COMPLETE)
	{
		TSK_HighFrequencyTask();
	}
	
}

void PWM_TIM_UP_IRQHandler(timer_callback_args_t * p_args)
{
	if (p_args->event == TIMER_EVENT_TROUGH)
	{
	}
	
	if (p_args->event == TIMER_EVENT_CREST)
	{
		R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_02, BSP_IO_LEVEL_HIGH);
		R_IOPORT_PinWrite(g_ioport.p_ctrl, BSP_IO_PORT_13_PIN_02, BSP_IO_LEVEL_LOW);
		
		ICS_TIMx_UP_IRQHandler(&PWM_Handle_M1);
	}
}

void HALL_TIM_UP_CC_IRQHandler(timer_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        switch (p_args->event)
        {
            case TIMER_EVENT_CYCLE_END:
                 HALL_TIMx_UP_IRQHandler(&HALL_M1);
                break;
            case TIMER_EVENT_CAPTURE_A:
                HALL_TIMx_CC_IRQHandler(&HALL_M1,&p_args->capture);
                break;
            case TIMER_EVENT_CAPTURE_B:
                break;
            case TIMER_EVENT_TROUGH:
                break;
        }
    }
}   /* End of function adc_eoc0_isr */