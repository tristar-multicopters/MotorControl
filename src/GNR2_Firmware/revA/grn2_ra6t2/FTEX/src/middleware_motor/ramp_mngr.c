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


/* 
    It reset the state variable to zero.
*/
void RampMngr_Init(RampMngr_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->wStateValue = 0;
    pHandle->wTargetFinal = 0;
    pHandle->wRampRemainingStep = 0u;
    pHandle->wIncDecAmount = 0;
}

/* 
    Exec the ramp calculations and returns the current value of the
    state variable.
*/
int32_t RampMngr_Calc(RampMngr_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int32_t ret_val;
    int32_t current_ref;

    current_ref = pHandle->wStateValue;

    /* Update the variable and terminates the ramp if needed. */
    if (pHandle->wRampRemainingStep > 1u)
    {
        /* Increment/decrement the reference value. */
        current_ref += pHandle->wIncDecAmount;

        /* Decrement the number of remaining steps */
        pHandle->wRampRemainingStep --;
    }
    else if (pHandle->wRampRemainingStep == 1u)
    {
        /* Set the backup value of wTargetFinal. */
        current_ref = pHandle->wTargetFinal * (int32_t)(pHandle->wScalingFactor);
        pHandle->wRampRemainingStep = 0u;
    }
    else
    {
        /* Do nothing. */
    }

    pHandle->wStateValue = current_ref;

    ret_val = pHandle->wStateValue / (int32_t)(pHandle->wScalingFactor);


    return ret_val;
}

/* 
    Setup the ramp to be executed
*/
bool RampMngr_ExecRamp(RampMngr_Handle_t * pHandle, int32_t wTargetFinal, uint32_t wSlopePerSecond)
{
    ASSERT(pHandle != NULL);
    int32_t aux1;
    int32_t current_ref;
    uint32_t wSlopePerControlPeriod;
    bool retVal = true;

    current_ref = pHandle->wStateValue / (int32_t)(pHandle->wScalingFactor);

    if (wSlopePerSecond == 0u)
    {
        pHandle->wStateValue = wTargetFinal * (int32_t)(pHandle->wScalingFactor);
        pHandle->wRampRemainingStep = 0u;
        pHandle->wIncDecAmount = 0;
    }
    else
    {
        /* Store the wTargetFinal to be applied in the last step */
        pHandle->wTargetFinal = wTargetFinal;

        /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
        the reference value at each CalcTorqueReference. */
        wSlopePerControlPeriod = (uint32_t) ((wSlopePerSecond*pHandle->wScalingFactor)/pHandle->wFrequencyHz);
        pHandle->wIncDecAmount = (wTargetFinal - current_ref) > 0 ? (int32_t) wSlopePerControlPeriod : (int32_t) -wSlopePerControlPeriod;
        
        /* Compute the (wRampRemainingStep) number of steps remaining to complete
        the ramp. */
        aux1 = (abs(wTargetFinal - current_ref) ) * (int32_t)(pHandle->wScalingFactor);
        aux1 /= (int32_t) wSlopePerControlPeriod;
        pHandle->wRampRemainingStep = (uint32_t) aux1;
        pHandle->wRampRemainingStep++;
    }

    return retVal;
}

/* 
    Returns the current value of the state variable.
*/
int32_t RampMngr_GetValue(RampMngr_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int32_t ret_val;
    ret_val = pHandle->wStateValue / (int32_t)(pHandle->wScalingFactor);
    return ret_val;
}

/* 
    Check if the settled ramp has been completed.
*/
bool RampMngr_IsRampCompleted(RampMngr_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool retVal = false;
    if (pHandle->wRampRemainingStep == 0u)
    {
        retVal = true;
    }
    return retVal;
}

/* 
    Stop the execution of the ramp keeping the last reached value.
*/
void RampMngr_StopRamp(RampMngr_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->wRampRemainingStep = 0u;
    pHandle->wIncDecAmount = 0;
}


