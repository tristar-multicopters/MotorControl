/**
  ******************************************************************************
  * @file    ramp_mngr.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file provides firmware functions that implement the features
  *          of the Ramp Manager component of the motor control application.
  *
  ******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RAMPMNGR_H
#define __RAMPMNGR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "ASSERT_FTEX.h"
#include "stdlib.h"

// Ramp types. If a ramp is added or remove, the define RAMP_NUMBER should be adjusted accordingly.
typedef enum
{
    RAMP_TORQUE     = 0,
    RAMP_SPEED      = 1,
} RampMngrTypes_t;

typedef struct
{
    uint32_t wFrequencyHz;             /*!< Execution frequency expressed in Hz */
    int32_t  wTargetFinal;             /*!< Value to be applied in the last step.*/
    int32_t  wStateValue;                     /*!< Current state variable multiplied by wScalingfactor.*/
    uint32_t wRampRemainingStep;       /*!< Number of steps remaining to complete the
                                         ramp.*/
    int32_t  wIncDecAmount;            /*!< Increment/decrement amount to be applied to
                                         the reference value at each RampMngr_Calc.*/
    uint32_t wScalingFactor;           /*!< Scaling factor between output value and
                                         its internal representation.*/

} RampMngr_Handle_t;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It reset the state variable to zero.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval none.
  */
void RampMngr_Init(RampMngrTypes_t chosenRamp);

/**
  * @brief  Exec the ramp calculations and returns the current value of the
            state variable.
            It must be called at fixed interval defined in the hExecFreq.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
int32_t RampMngr_Calc(RampMngrTypes_t chosenRamp);

/**
  * @brief  Setup the ramp to be executed
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @param  hTargetFinal (signed 32bit) final value of state variable at the end
  *         of the ramp.
  * @param  wSlopePerSecond the slope of the ramp expressed in
  *         unit per second. It is possible to set 0 to perform an instantaneous
  *         change in the value.
  * @retval bool It returns true is command is valid, false otherwise
  */
bool RampMngr_ExecRamp(RampMngrTypes_t chosenRamp, int32_t wTargetFinal, uint32_t wSlopePerSecond);

/**
  * @brief  Returns the current value of the state variable.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
int32_t RampMngr_GetValue(RampMngrTypes_t chosenRamp);

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  pHandle related Handle of struct RampMngr_Handle_t.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool RampMngr_IsRampCompleted(RampMngrTypes_t chosenRamp);

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  pHandle related Handle of struct RampMngr_Handle_t.
  * @retval none
  */
void RampMngr_StopRamp(RampMngrTypes_t chosenRamp);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __RAMPMNGR_H */



