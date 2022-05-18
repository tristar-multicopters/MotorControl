
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  FTEX inc
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "parameters_conversion.h"

#include "ics_ra6t2_pwm_curr_fdbk.h"

const ICS_Params_t ICS_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
	.pADCHandle = &g_adc,
  .ADCChannelIa = ADC_CHANNEL_4,
	.ADCChannelIb = ADC_CHANNEL_2,
	.ADCGroupMask = ADC_GROUP_MASK_0,

/* PWM generation parameters --------------------------------------------------*/
	.pThreePhaseHandle  =	&g_three_phase0,
  .RepetitionCounter 	=	REP_COUNTER,


/* Emergengy signal initialization ----------------------------------------*/
  .BKIN2Mode           = INT_MODE,

};

