/**
  * @file    ramps.c
  * @brief   This module handles acceleration/deceleration ramps
  *
*/

#include "ramps.h"
#include <math.h>

/**
  * @brief  Dynamic deceleration ramp. 
  * Linear ramp thats reduce the amount of deceleration according
  * to the current speed of the bike. The fastest the bike goes,
  * the slower the deceleration. The ramp parameters can be change
  * in the vc_parameters_bike.h under the "Dynamic Deceleration Ramp Params"
  * @param currentSpeed : current bike speed during the ramp filtering
  * @param input : Torque power delivered before the ramp filtering
  * @retval Torque power delivered after the ramp filtering                                                                                    
  */
int16_t Ramps_DynamicDecelerationRamp(float currentSpeed, int16_t input);

/**
  * @brief  High speed power limiting ramp. 
  * Linear ramp thats reduce the amount of power delivered according
  * to the current speed of the bike. The fastest the bike goes,
  * the less power we deliver. The ramp parameters can be change
  * in the vc_parameters_bike.h under the "High Speed Power Limiting Ramp Param"
  * @param currentSpeed : current bike speed during the ramp filtering
  * @param input : Torque power delivered before the ramp filtering
  * @retval Torque power delivered after the ramp filtering                                                                                    
  */
int16_t Ramps_HighSpeedPowerLimitingRamp(float currentSpeed, int16_t input);


/**
  * @brief  Apply the ramp to power delivered
*/ 
int16_t Ramps_ApplyRamp(RampType_t rampType, float currentSpeed, int16_t input)
{
    switch (rampType)
    {
        case NO_RAMP_SELECTED:
            return input;
        case DYNAMIC_DECELERATION_RAMP:
            return Ramps_DynamicDecelerationRamp(currentSpeed, input);
        case HIGH_SPEED_POWER_LIMITING_RAMP:
            return Ramps_HighSpeedPowerLimitingRamp(currentSpeed, input);
        default:
            return input;
    }
}

/**
  * @brief  Dynamic deceleration ramp. 
  * Linear ramp thats reduce the amount of deceleration according
  * to the current speed of the bike. The fastest the bike goes,
  * the slower the deceleration. The ramp parameters can be change
*/
int16_t Ramps_DynamicDecelerationRamp(float currentSpeed, int16_t input)
{
    static int16_t prevInput = 0;
    int16_t output = 0;

    // Calculate the minimum amount of power we can deliver so we do not decelerate too fast
    float currentDecelerationAmplitude = currentSpeed * (DYNAMIC_DECEL_RAMP_POWER_MIN_SPEED - DYNAMIC_DECEL_RAMP_POWER_MAX_SPEED)
                                         /(DYNAMIC_DECEL_RAMP_END - DYNAMIC_DECEL_RAMP_START) + DYNAMIC_DECEL_RAMP_POWER_MAX_SPEED;

    float minOutput = prevInput * (1 - fabsf(currentDecelerationAmplitude/DYNAMIC_DECEL_RAMP_POWER_MAX_SPEED));

    // If the input is lower than the minimum output, we return the minimum output
    if(input < minOutput) output = (int16_t)minOutput;
    else output = input;

    prevInput = output;
    return output;
}

/**
  * @brief  High speed power limiting ramp. 
  * Linear ramp thats reduce the amount of power delivered according
  * to the current speed of the bike. The fastest the bike goes,
  * the less power we deliver. The ramp parameters can be change
  * in the vc_parameters_bike.h under the "High Speed Power Limiting Ramp Param"
*/
int16_t Ramps_HighSpeedPowerLimitingRamp(float currentSpeed, int16_t input)
{
    if(currentSpeed < HIGH_SPEED_POWER_LIMITING_RAMP_START)
        return input;

    if(currentSpeed > HIGH_SPEED_POWER_LIMITING_RAMP_END)
        return (int16_t)((float)input * ((float)HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MIN_SPEED/100.0));        

    // We calculate the amount (% of max power) that needs to be trimmed off from the input 
    int16_t output = 0;
    float decelerationRampFactor = (HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MAX_SPEED - HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MIN_SPEED)
                                    /(HIGH_SPEED_POWER_LIMITING_RAMP_END - HIGH_SPEED_POWER_LIMITING_RAMP_START);
    output = (int16_t)(input - fabsf(((decelerationRampFactor * currentSpeed) / 100) * input)); 
    
    return output;
}
