/**
  * @file    ramps.c
  * @brief   This module handles acceleration/deceleration ramps
  *
*/

#include "ramps.h"
#include <math.h>

DynamicDecelerationRampParameters_t dynamicDecelerationParams = 
{
    .rampStart = DYNAMIC_DECEL_RAMP_START,
    .rampEnd = DYNAMIC_DECEL_RAMP_END,
    .powerMinSpeed = DYNAMIC_DECEL_RAMP_POWER_MIN_SPEED,
    .powerMaxSpeed = DYNAMIC_DECEL_RAMP_POWER_MAX_SPEED
};

HighSpeedPowerLimitingRampParameters_t highSpeedPowerLimitingParams = 
{
    .rampStart = HIGH_SPEED_POWER_LIMITING_RAMP_START,        
    .rampEnd = HIGH_SPEED_POWER_LIMITING_RAMP_END,          
    .powerMinSpeed = HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MIN_SPEED,     
    .powerMaxSpeed = HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MAX_SPEED     
};

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
  * @brief This function ensures that the value of torque,
  *        calculated by a ramp, is a positive value.
  * @param originalInput : Original input provided in the ramp function
  * @param torqueToValidate : Value that needs > 0 validation
  * @retval If torque to validate > 0, return torqueToValidate, otherwise originalInput
  */
int16_t NegativeTorqueProtection(int16_t originalInput, int16_t torqueToValidate);


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
  * @brief  Set dynamic deceleration ramp parameters
  */
void Ramps_SetDynamicDecelerationRampParameters(float rampStart, float rampEnd, float powerMinSpeed, float powerMaxSpeed)
{
    dynamicDecelerationParams.rampStart = rampStart;
    dynamicDecelerationParams.rampEnd = rampEnd;
    dynamicDecelerationParams.powerMinSpeed = powerMinSpeed;
    dynamicDecelerationParams.powerMaxSpeed = powerMaxSpeed;
}

/**
  * @brief  Set high speed power limiting ramp parameters
  */
void Ramps_SetHighSpeedPowerLimitingRampParameters(float rampStart, float rampEnd, float powerMinSpeed, float powerMaxSpeed)
{
    highSpeedPowerLimitingParams.rampStart = rampStart;
    highSpeedPowerLimitingParams.rampEnd = rampEnd;
    highSpeedPowerLimitingParams.powerMinSpeed = powerMinSpeed;
    highSpeedPowerLimitingParams.powerMaxSpeed = powerMaxSpeed;
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
    float currentDecelerationAmplitude = currentSpeed * (dynamicDecelerationParams.powerMinSpeed - dynamicDecelerationParams.powerMaxSpeed)
                                         /(dynamicDecelerationParams.rampEnd - dynamicDecelerationParams.rampStart) + dynamicDecelerationParams.powerMaxSpeed;

    float minOutput = prevInput * (1 - fabsf(currentDecelerationAmplitude/dynamicDecelerationParams.powerMaxSpeed));

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
    int16_t output = 0;

    // Check if the speed is in the left most constant part of the ramp
    if(currentSpeed < highSpeedPowerLimitingParams.rampStart)
    {
        output = (int16_t)((float)input * (highSpeedPowerLimitingParams.powerMinSpeed/FLOAT_PERCENTAGE_DIVIDER));
        return NegativeTorqueProtection(input, output);
    }

    // Check if the speed is in the right most constant part of the ramp
    if(currentSpeed > highSpeedPowerLimitingParams.rampEnd)
    {
        output = (int16_t)((float)input * (highSpeedPowerLimitingParams.powerMaxSpeed/FLOAT_PERCENTAGE_DIVIDER));
        return NegativeTorqueProtection(input, output);
    }

    // Calculate the variation within the linear ramp part
    float powerLimiterVariation = (highSpeedPowerLimitingParams.powerMaxSpeed - highSpeedPowerLimitingParams.powerMinSpeed)
                                  /(highSpeedPowerLimitingParams.rampEnd - highSpeedPowerLimitingParams.rampStart);

    // Get the current speed value within the linear ramp part
    float speedInRamp = currentSpeed - highSpeedPowerLimitingParams.rampStart;

    // Calculate the linear power limiting factor
    float powerLimiterFactor = powerLimiterVariation * speedInRamp + highSpeedPowerLimitingParams.powerMinSpeed;

    // Apply the limiting factor to the input
    output = (int16_t)((float)input * powerLimiterFactor/FLOAT_PERCENTAGE_DIVIDER);

    return NegativeTorqueProtection(input, output);
}

/**
  * @brief This function ensures that the value of torque,
  *        calculated by a ramp, is a positive value.
*/
int16_t NegativeTorqueProtection(int16_t originalInput, int16_t torqueToValidate)
{
    if(torqueToValidate < 0) return originalInput;
    return torqueToValidate;
}
