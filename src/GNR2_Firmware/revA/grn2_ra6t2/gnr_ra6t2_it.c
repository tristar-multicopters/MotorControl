/**
  * @file    gnr_ra6t2_it.c
  * @brief   Interrupt Service Routines.
*/

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "bsp_pin_cfg.h"


/**
  * @brief  Interrupt routine of ADC hardware.
  * @param  p_args: ADC callback function arguments.
  */
void CS_ADC_IRQHandler(adc_callback_args_t * p_args)
{
	if (p_args->event == ADC_EVENT_SCAN_COMPLETE && p_args->group_mask == ADC_GROUP_MASK_0)
	{
		/* Run motor control high frequency task */
		MC_HighFrequencyTask();
	}

}

/**
  * @brief  Interrupt routine of PWM timer hardware.
  * @param  p_args: Timer callback function arguments.
  */
void PWM_TIM_UP_IRQHandler(timer_callback_args_t * p_args)
{
	if (p_args->event == TIMER_EVENT_TROUGH)
	{
	}

	if (p_args->event == TIMER_EVENT_CREST)
	{
		/* Run motor control timer update routine */
		PWMInsulCurrSensorFdbk_TIMx_UP_IRQHandler(&PWMHandleM1);
	}
}

/**
  * @brief  Interrupt routine when POEG pin is pulled down. It means an overcurrent
	*					condition is detected and consequently PWM outputs were forcefully stopped.
  * @param  p_args: POEG callback function arguments, not used.
  */
void PWM_TIM_BRK_IRQHandler(poeg_callback_args_t * p_args)
{
	if(NULL != p_args)
  {
	  /* Stop POEG module so it does not reenter this interrupt twice */
		R_POEG_Close(PWM_Handle_M1.pParams_str->pPOEGHandle->p_ctrl);

		/* Run motor control PWM break routine */
		ICS_BRK2_IRQHandler(&PWM_Handle_M1);
  }
}

/**
  * @brief  Interrupt routine of hall timer hardware.
  * @param  p_args: Timer callback function arguments.
  */
void HALL_TIM_UP_CC_IRQHandler(timer_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        switch (p_args->event)
        {
            case TIMER_EVENT_CYCLE_END:
                HallPosSensor_TIMx_UP_IRQHandler(&HallPosSensorM1);
                break;
            case TIMER_EVENT_CAPTURE_A:
                HallPosSensor_TIMx_CC_IRQHandler(&HallPosSensorM1,&p_args->capture);
                break;
            case TIMER_EVENT_CAPTURE_B:
                break;
            case TIMER_EVENT_TROUGH:
                break;
        }
    }
}
