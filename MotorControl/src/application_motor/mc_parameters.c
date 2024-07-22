/**
  * @file    mc_parameters.c
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "gnr_main.h"
#include "board_hardware.h"
#include "parameters_conversion.h"

#include "ics_ra6t2_pwm_curr_fdbk.h"

const PWMInsulCurrSensorFdbkParams_t PWMICSParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
    .pADCHandle = CURRENT_SENSOR_ADC_HANDLE_ADDRESS,
    .ADCGroupMask = CURRENT_SENSOR_ADC_GROUP_MASK,
    .ADCChannelIa = CURRENT_SENSOR_IA_ANALOG_CHANNEL,
    .ADCChannelIb = CURRENT_SENSOR_IB_ANALOG_CHANNEL,

/* PWM generation parameters --------------------------------------------------*/
    .pThreePhaseHandle  =    PWM_THREE_PHASE_HANDLE_ADDRESS,
    .bRepetitionCounter     =    REP_COUNTER,
    
/* PWM break input parameters --------------------------------------------------*/
    .pPOEGHandle = PWM_POEG0_HANDLE_ADDRESS,
    
};

