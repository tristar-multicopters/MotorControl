/**
    ******************************************************************************
    * @file      ramp_mngr.c
    * @author    Sami Bouzid, FTEX inc
    * @brief     This file provides firmware functions that implement the features
    *            of the Ramp Manager component of the motor control application.
    *
    ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "ramp_mngr.h"
#include "parameters_conversion.h"

#define NUMBER_RAMPS        2

RampMngr_Handle_t RampManagers[NUMBER_RAMPS];


/* 
    It reset the state variable to zero.
*/
void RampMngr_Init(uint8_t chosenRamp)
{
    ASSERT(chosenRamp < NUMBER_RAMPS);
    RampManagers[chosenRamp].wScalingFactor = INT16_MAX;
    RampManagers[chosenRamp].wFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE;
    RampManagers[chosenRamp].wStateValue = 0;
    RampManagers[chosenRamp].wTargetFinal = 0;
    RampManagers[chosenRamp].wRampRemainingStep = 0u;
    RampManagers[chosenRamp].wIncDecAmount = 0;
}

/* 
    Exec the ramp calculations and returns the current value of the
    state variable.
*/
int32_t RampMngr_Calc(uint8_t chosenRamp)
{
    ASSERT(chosenRamp < NUMBER_RAMPS);
    int32_t ret_val;
    int32_t current_ref;

    current_ref = RampManagers[chosenRamp].wStateValue;

    /* Update the variable and terminates the ramp if needed. */
    if (RampManagers[chosenRamp].wRampRemainingStep > 1u)
    {
        /* Increment/decrement the reference value. */
        current_ref += RampManagers[chosenRamp].wIncDecAmount;

        /* Decrement the number of remaining steps */
        RampManagers[chosenRamp].wRampRemainingStep --;
    }
    else if (RampManagers[chosenRamp].wRampRemainingStep == 1u)
    {
        /* Set the backup value of wTargetFinal. */
        current_ref = RampManagers[chosenRamp].wTargetFinal * (int32_t)(RampManagers[chosenRamp].wScalingFactor);
        RampManagers[chosenRamp].wRampRemainingStep = 0u;
    }
    else
    {
        /* Do nothing. */
    }

    RampManagers[chosenRamp].wStateValue = current_ref;

    ret_val = RampManagers[chosenRamp].wStateValue / (int32_t)(RampManagers[chosenRamp].wScalingFactor);


    return ret_val;
}

/* 
    Setup the ramp to be executed
*/
bool RampMngr_ExecRamp(uint8_t chosenRamp, int32_t wTargetFinal, uint32_t wSlopePerSecond)
{
    ASSERT(chosenRamp < NUMBER_RAMPS);
    int32_t aux1;
    int32_t current_ref;
    uint32_t wSlopePerControlPeriod;
    bool retVal = true;

    current_ref = RampManagers[chosenRamp].wStateValue / (int32_t)(RampManagers[chosenRamp].wScalingFactor);

    if (wSlopePerSecond == 0u)
    {
        RampManagers[chosenRamp].wStateValue = wTargetFinal * (int32_t)(RampManagers[chosenRamp].wScalingFactor);
        RampManagers[chosenRamp].wRampRemainingStep = 0u;
        RampManagers[chosenRamp].wIncDecAmount = 0;
    }
    else
    {
        /* Store the wTargetFinal to be applied in the last step */
        RampManagers[chosenRamp].wTargetFinal = wTargetFinal;

        /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
        the reference value at each CalcTorqueReference. */
        wSlopePerControlPeriod = (uint32_t) ((wSlopePerSecond*RampManagers[chosenRamp].wScalingFactor)/RampManagers[chosenRamp].wFrequencyHz);
        RampManagers[chosenRamp].wIncDecAmount = (wTargetFinal - current_ref) > 0 ? (int32_t) wSlopePerControlPeriod : (int32_t) -wSlopePerControlPeriod;
        
        /* Compute the (wRampRemainingStep) number of steps remaining to complete
        the ramp. */
        aux1 = (abs(wTargetFinal - current_ref)) * (int32_t)(RampManagers[chosenRamp].wScalingFactor);
        aux1 /= (int32_t) wSlopePerControlPeriod;
        RampManagers[chosenRamp].wRampRemainingStep = (uint32_t) aux1;
        RampManagers[chosenRamp].wRampRemainingStep++;
    }

    return retVal;
}

/* 
    Returns the current value of the state variable.
*/
int32_t RampMngr_GetValue(uint8_t chosenRamp)
{
    ASSERT(chosenRamp < NUMBER_RAMPS);
    int32_t ret_val;
    ret_val = RampManagers[chosenRamp].wStateValue / (int32_t)(RampManagers[chosenRamp].wScalingFactor);
    return ret_val;
}

/* 
    Check if the settled ramp has been completed.
*/
bool RampMngr_IsRampCompleted(uint8_t chosenRamp)
{
    ASSERT(chosenRamp < NUMBER_RAMPS);
    bool retVal = false;
    if (RampManagers[chosenRamp].wRampRemainingStep == 0u)
    {
        retVal = true;
    }
    return retVal;
}

/* 
    Stop the execution of the ramp keeping the last reached value.
*/
void RampMngr_StopRamp(uint8_t chosenRamp)
{
    ASSERT(chosenRamp < NUMBER_RAMPS);
    RampManagers[chosenRamp].wRampRemainingStep = 0u;
    RampManagers[chosenRamp].wIncDecAmount = 0;
}


