/**
  ******************************************************************************
  * @file    parameters_conversion_g4xx.h
  * @author  FTEX inc
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32G4 Family.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_RA6T2_H
#define __PARAMETERS_CONVERSION_RA6T2_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      240000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz  120
#define ADC_CLK_MHz     60
//#define HALL_TIM_CLK    170000000uL
//#define APB1TIM_FREQ 170000000uL

//#define ADC_TRIG_CONV_LATENCY_CYCLES 3.5
//#define ADC_SAR_CYCLES 12.5

//#define M1_VBUS_SW_FILTER_BW_FACTOR     6u



#endif /*__PARAMETERS_CONVERSION_RA6T2_H*/


