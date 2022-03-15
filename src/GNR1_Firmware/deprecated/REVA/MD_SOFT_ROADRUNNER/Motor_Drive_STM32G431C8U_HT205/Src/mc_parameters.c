
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
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
#include "main.h"
#include "parameters_conversion.h"

#include "r3_2_g4xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ADC1,
  .ADCx_2           = ADC2,
  /* Motor Control Kit config */
  .ADCConfig1 = { MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCConfig2 = { MC_ADC_CHANNEL_12<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_12<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_12<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , MC_ADC_CHANNEL_12<<ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCDataReg1 = { &ADC1->JDR1
                 , &ADC1->JDR1
                 , &ADC1->JDR1
                 , &ADC1->JDR1
                 , &ADC1->JDR1
                 , &ADC1->JDR1
                 },
  .ADCDataReg2 = { &ADC2->JDR1
                 , &ADC2->JDR1
                 , &ADC2->JDR1
                 , &ADC2->JDR1
                 , &ADC2->JDR1
                 , &ADC2->JDR1
                 },
  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .TIMx               = TIM1,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = INT_MODE,

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMPParams     = MC_NULL,
/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPASelection     = COMP1,
  .CompOCPAInvInput_MODE = INT_MODE,
  .CompOCPBSelection     = COMP2,
  .CompOCPBInvInput_MODE = INT_MODE,
  .CompOCPCSelection     = COMP4,
  .CompOCPCInvInput_MODE = INT_MODE,
  .DAC_OCP_ASelection    = DAC3,
  .DAC_OCP_BSelection    = DAC3,
  .DAC_OCP_CSelection    = DAC3,
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_1,
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_2,
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_2,

  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  60000,
  .DAC_OVP_Threshold =  60000,
	
/* OCP Protection delay --------------------------------------------------------------*/	

	.TIM_OCPa = TIM2,
	.TIM_OCPb = TIM3,
	.TIM_OCPc = TIM4,
	
	.Tdelay_OCP = 1000,
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
