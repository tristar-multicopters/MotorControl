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

// all floats

typedef struct DynamicDecelerationRampParameters_t
{
    float rampStart;        // Min speed(km/h) where the dynamic deceleration ramp starts
    float rampEnd;          // Max speed(km/h) where the dynamic deceleration ramp ends
    float powerMinSpeed;    // Dynamic deceleration ramp value(in % of MAX power) at max speed(km/h)
    float powerMaxSpeed;    // Dynamic deceleration ramp value(in % of MAX power) at min speed(km/h)
} DynamicDecelerationRampParameters_t;

typedef struct HighSpeedPowerLimitingRampParameters_t
{
    float rampStart;        // Min speed(km/h) where the dynamic deceleration ramp starts
    float rampEnd;          // Max speed(km/h) where the dynamic deceleration ramp ends
    float powerMinSpeed;    // Power allowed (in % of MAX power) at ramp start min speed(km/h)
    float powerMaxSpeed;    // Power allowed (in % of MAX power) at ramp end max speed(km/h)
} HighSpeedPowerLimitingRampParameters_t;

/**
  * @brief  Apply the ramp to power delivered
  * @param  rampType : Type of ramp that needs to be applied
  * @param currentSpeed : current bike speed during the ramp filtering
  * @param input : Torque power delivered before the ramp filtering
  * @retval Torque power delivered after the ramp filtering                                                                                    
  */
int16_t Ramps_ApplyRamp(RampType_t rampType, float currentSpeed, int16_t input);

/**
  * @brief  Set dynamic deceleration ramp parameters
  * @param  rampStart : Ramp start value to set
  * @param  rampEnd : Ramp end value to set
  * @param  powerMinSpeed : Power min speed value to set
  * @param  powerMaxSpeed : Power max speed value to set
  * @retval None                                                                                    
  */
void Ramps_SetDynamicDecelerationRampParameters(float rampStart, float rampEnd, float powerMinSpeed, float powerMaxSpeed);

/**
  * @brief  Set high speed power limiting ramp parameters
  * @param  rampStart : Ramp start value to set
  * @param  rampEnd : Ramp end value to set
  * @param  powerMinSpeed : Power min speed value to set
  * @param  powerMaxSpeed : Power max speed value to set
  * @retval None                                                                                    
  */
void Ramps_SetHighSpeedPowerLimitingRampParameters(float rampStart, float rampEnd, float powerMinSpeed, float powerMaxSpeed);

#endif /*__RAMPS_H*/
