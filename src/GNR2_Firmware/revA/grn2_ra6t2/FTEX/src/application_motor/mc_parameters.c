/**
  * @file    mc_parameters.c
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "parameters_conversion.h"

#include "ics_ra6t2_pwm_curr_fdbk.h"

const PWMInsulCurrSensorFdbkParams_t PWMICSParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
	.pADCHandle = &g_adc,
    .ADCChannelIa = ADC_CHANNEL_4,
	.ADCChannelIb = ADC_CHANNEL_2,
	.ADCGroupMask = ADC_GROUP_MASK_0,

/* PWM generation parameters --------------------------------------------------*/
	.pThreePhaseHandle  =	&g_three_phase0,
  .bRepetitionCounter 	=	REP_COUNTER,
	
/* PWM break input parameters --------------------------------------------------*/
	.pPOEGHandle = &g_poeg1,
	
};

