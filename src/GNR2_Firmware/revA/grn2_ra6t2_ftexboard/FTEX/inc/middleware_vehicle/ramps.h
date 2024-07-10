/**
  * @file    ramps.h
  * @brief   This module handles power enable pin
  *
*/ 
#ifndef __RAMPS_H
#define __RAMPS_H

#include "ASSERT_FTEX.h"
#include "stdint.h"
#include "stdbool.h"
#include "vc_parameters.h"

#define FLOAT_PERCENTAGE_DIVIDER 100.0f

// List of all available ramps
typedef enum RampType_t
{
    NO_RAMP_SELECTED                    = 0,
    DYNAMIC_DECELERATION_RAMP           = 1,
    HIGH_SPEED_POWER_LIMITING_RAMP      = 2
} RampType_t;

/**
  * @brief  Apply the ramp to power delivered
  * @param  rampType : Type of ramp that needs to be applied
  * @param currentSpeed : current bike speed during the ramp filtering
  * @param input : Torque power delivered before the ramp filtering
  * @retval Torque power delivered after the ramp filtering                                                                                    
  */
int16_t Ramps_ApplyRamp(RampType_t rampType, float currentSpeed, int16_t input);

#endif /*__RAMPS_H*/
