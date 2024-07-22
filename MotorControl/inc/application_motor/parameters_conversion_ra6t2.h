/**
  * @file    parameters_conversion_g4xx.h
  * @brief   This file contains the definitions needed to convert MC application parameters
  *          so as to target the STM32G4 Family.
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_RA6T2_H
#define __PARAMETERS_CONVERSION_RA6T2_H

#include "drive_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ         240000000uL
#define PWM_TIM_CLK_MHz     120
#define ADC_CLK_MHz         60
#define HALL_TIM_CLK        120000000uL


#endif /*__PARAMETERS_CONVERSION_RA6T2_H*/
