/**
  * @file    mc_state_machine.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Motor Control State Machine component of the Motor Control application:
  *
  *           * Check that transition from one state to another is legal
  *           * Handle the fault processing
  *           * Provide accessor to State machine internal state
  *           * Provide accessor to error state
  *
    */

/* Includes ------------------------------------------------------------------*/
#include "mc_state_machine.h"
#include "gnr_parameters.h"

#define ERROR_TIMER     2000    //~2 secs, timer used to reset timer

MotorStateMachineHandle_t MCStateMachine;

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param pHandle pointer on the component instance to initialize.
  * @retval none.
  */
void MCStateMachine_Init()
{

    MCStateMachine.bState = M_IDLE;
    MCStateMachine.wCriticalFaultNow = MC_NO_FAULT;
    MCStateMachine.wCriticalFaultOccurred = MC_NO_FAULT;
    MCStateMachine.wCurrentErrorsNow = MC_NO_ERROR;
    MCStateMachine.wOccurredErrors = MC_NO_ERROR;
    MCStateMachine.hErrorProcessingTimer = 0;
    MCStateMachine.wWarnings = MC_NO_WARNING;
    
}

/**
  * @brief It submits the request for moving the state machine into the state
  *        specified by bState (FAULT_NOW and FAUL_OVER are not handled by this
  *        method). Accordingly with the current state, the command is really
  *        executed (state machine set to bState) or discarded (no state
  *        changes).
  *        If requested state can't be reached the return value is false and the
  *        MC_SW_ERROR is raised, but if requested state is IDLE_START,
  *        IDLE_ALIGNMENT or ANY_STOP, that corresponds with the user actions:
  *        Start Motor, Encoder Alignemnt and Stop Motor, the MC_SW_ERROR is
  *        not raised.
  * @param pHandle pointer of type  MotorStateMachineHandle_t.
  * @param bState New requested state
  * @retval bool It returns true if the state has been really set equal to
  *         bState, false if the requested state can't be reached
  */
bool MCStateMachine_NextState(MotorState_t bState)
{
  bool bChangeState = false;
  MotorState_t bCurrentState = MCStateMachine.bState;
  MotorState_t bNewState = bCurrentState;

  switch (bCurrentState)
  {
    case M_ICLWAIT:
      if (bState == M_IDLE)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
    case M_IDLE:
      if ((bState == M_IDLE_START) || (bState == M_IDLE_ALIGNMENT)
           || (bState == M_ICLWAIT)  || (bState == M_AUTOTUNE_ENTER_IDENTIFICATION))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_IDLE_ALIGNMENT:
      if ((bState == M_ANY_STOP) || (bState == M_ALIGN_CHARGE_BOOT_CAP)
           || (bState == M_ALIGN_OFFSET_CALIB))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGN_CHARGE_BOOT_CAP:
      if ((bState == M_ALIGN_OFFSET_CALIB) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGN_OFFSET_CALIB:
      if ((bState == M_ALIGN_CLEAR) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGN_CLEAR:
      if ((bState == M_ALIGNMENT) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGNMENT:
      if (bState == M_ANY_STOP)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_IDLE_START:
      if ((bState == M_ANY_STOP) || (bState == M_CHARGE_BOOT_CAP) ||
           (bState == M_START) ||
           (bState == M_OFFSET_CALIB) || (bState == M_IDLE_ALIGNMENT))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_CHARGE_BOOT_CAP:
      if ((bState == M_OFFSET_CALIB) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_OFFSET_CALIB:
      if ((bState == M_CLEAR) || (bState == M_ANY_STOP) || (bState == M_WAIT_STOP_MOTOR))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

     case M_WAIT_STOP_MOTOR:
      if ((bState == M_CLEAR) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_CLEAR:
      if ((bState == M_START) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_START:
      if ((bState == M_SWITCH_OVER) || (bState == M_ANY_STOP) || (bState == M_START_RUN))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_SWITCH_OVER:
      if ((bState == M_START) || (bState == M_ANY_STOP) || (bState == M_START_RUN))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_START_RUN:
      if ((bState == M_RUN) || (bState == M_ANY_STOP))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_RUN:
      if (bState == M_ANY_STOP)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ANY_STOP:
      if (bState == M_STOP)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_STOP:
      if (bState == M_STOP_IDLE)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_STOP_IDLE:
      if ((bState == M_IDLE) || (bState == M_ICLWAIT))
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
   #if AUTOTUNE_ENABLE
    case M_AUTOTUNE_ENTER_IDENTIFICATION:
      if (bState == M_AUTOTUNE_IDENTIFICATION)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
      
    case M_AUTOTUNE_IDENTIFICATION:
      if (bState == M_AUTOTUNE_ANY_STOP_IDENTIFICATION)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
      
    case M_AUTOTUNE_ANY_STOP_IDENTIFICATION:
      if (bState == M_AUTOTUNE_STOP_IDENTIFICATION)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
      
    case M_AUTOTUNE_STOP_IDENTIFICATION:
      if (bState == M_IDLE)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
   #endif
    default:
      break;
  }

  if (bChangeState)
  {
    MCStateMachine.bState = bNewState;
  }
  else
  {
    if (!((bState == M_IDLE_START) || (bState == M_IDLE_ALIGNMENT)
            || (bState == M_ANY_STOP)))
    {
      /* If new state is not a user command START/STOP raise a software error */
      MCStateMachine_CriticalFaultProcessing(MC_SW_ERROR, 0u);
    }
  }

  return (bChangeState);
}

/**
  * @brief It clocks both HW and SW critical faults processing and update the state
  *        machine accordingly with hSetErrors, hResetErrors and present state.
  *        Refer to MotorState_t description for more information about fault states.
  * @param pHandle pointer of type  MotorStateMachineHandle_t
  * @param wSetErrors Bit field reporting critical faults currently present
  * @param wResetErrors Bit field reporting critical faults to be cleared
  * @retval MotorState_t New state machine state after fault processing
  */
MotorState_t MCStateMachine_CriticalFaultProcessing(uint32_t wSetErrors, uint32_t
                             wResetErrors)
{
  MotorState_t LocalState =  MCStateMachine.bState;

  /* Set current errors */
  MCStateMachine.wCriticalFaultNow = (MCStateMachine.wCriticalFaultNow | wSetErrors) & (~wResetErrors);
  MCStateMachine.wCriticalFaultOccurred |= wSetErrors;

  if (LocalState == M_FAULT_NOW)
  {
    if (MCStateMachine.wCriticalFaultNow == MC_NO_FAULT)
    {
      MCStateMachine.bState = M_FAULT_OVER;
      LocalState = M_FAULT_OVER;
    }
  }
  else
  {
    if (MCStateMachine.wCriticalFaultNow != MC_NO_FAULT)
    {
      MCStateMachine.bState = M_FAULT_NOW;
      LocalState = M_FAULT_NOW;
    }
  } 
  return (LocalState);
}

/**
  * @brief It sets errors
  * @param pHandle pointer of type  MotorStateMachineHandle_t
  * @param wSetErrors Bit field reporting errors currently present
  * @param wResetErrors Bit field reporting errors to be cleared
  * @retval none.
  */
void MCStateMachine_SetError(uint32_t wSetErrors, uint32_t wResetErrors)
{
    MCStateMachine.wCurrentErrorsNow = (MCStateMachine.wCurrentErrorsNow | wSetErrors) & (~wResetErrors);
    if (MCStateMachine.wCurrentErrorsNow)
    {
        MCStateMachine.wOccurredErrors = MCStateMachine.wCurrentErrorsNow;
    }   
}

/**
  * @brief It processes the error
  * @param pHandle pointer of type  MotorStateMachineHandle_t
  * @retval none.
  */
void MCStateMachine_ErrorProcessing()
{
    //if errors are cleared, wait until end of timer before you are able to push power
    if (MCStateMachine.wOccurredErrors && !MCStateMachine.wCurrentErrorsNow)
    {
        MCStateMachine.hErrorProcessingTimer++;
        if (MCStateMachine.hErrorProcessingTimer >= ERROR_TIMER)
        {
            MCStateMachine.hErrorProcessingTimer = 0;
            MCStateMachine.wOccurredErrors = MC_NO_ERROR;
        }
    }
}
    
/**
  * @brief It clocks both HW and SW warning processing
  * @param pHandle pointer of type  MotorStateMachineHandle_t
  * @param wSetWarnings Bit field reporting warnings currently present
  * @param wResetWarnings Bit field reporting warnings to be cleared
  * @retval none.
  */
void MCStateMachine_WarningHandling(uint32_t wSetWarnings, uint32_t  wResetWarnings)
{
    MCStateMachine.wWarnings = (MCStateMachine.wWarnings | wSetWarnings) & (~wResetWarnings);
}

/**
  * @brief  Returns the current state machine state
  * @param  pHandle pointer of type  MotorStateMachineHandle_t
  * @retval MotorState_t Current state machine state
  */
MotorState_t MCStateMachine_GetState()
{
  return (MCStateMachine.bState);
}

/**
  * @brief It reports to the state machine that the fault state has been
  *        acknowledged by the user. If the state machine is in FAULT_OVER state
  *        then it is moved into STOP_IDLE and the bit field variable containing
  *        information about the faults historically occurred is cleared.
  *        The method call is discarded if the state machine is not in FAULT_OVER
  * @param pHandle pointer of type  MotorStateMachineHandle_t
  * @retval bool true if the state machine has been moved to IDLE, false if the
  *        method call had no effects
  */
bool MCStateMachine_CriticalFaultAcknowledged()
{
  bool bToBeReturned = false;

  if (MCStateMachine.bState == M_FAULT_OVER)
  {
    MCStateMachine.bState = M_STOP_IDLE;
    MCStateMachine.wCriticalFaultOccurred = MC_NO_FAULT;
    bToBeReturned = true;
  }

  return (bToBeReturned);
}

/**
  * @brief It returns two 16 bit fields containing information about both critical faults
  *        currently present and critical faults historically occurred since the state
  *        machine has been moved into state
  * @param pHandle pointer of type  MotorStateMachineHandle_t.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present critical faults. In the least
  *         significant half are stored the information about the critical faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state
  */
uint64_t MCStateMachine_GetCriticalFaultState()
{
  uint64_t LocalFaultState;

  LocalFaultState = (uint64_t)(MCStateMachine.wCriticalFaultOccurred);
  LocalFaultState |= (uint64_t)(MCStateMachine.wCriticalFaultNow) << 32;

  return LocalFaultState;
}

/**
  * @brief It returns a 16 bit fields containing information about errors
  *        currently present 
  * @param pHandle pointer of type  MotorStateMachineHandle_t.
  * @retval uint32_t  a 16 bit field that shoing occurred errors
  */
uint32_t MCStateMachine_GetCurrentErrorState()
{
    return MCStateMachine.wCurrentErrorsNow;
}

/**
  * @brief It returns a boolean indicating whether the error is still processing
  * @param pHandle pointer of type  MotorStateMachineHandle_t.
  * @retval boolean indicating whether error is processing
  */
uint32_t MCStateMachine_GetOccurredErrorState()
{
    return MCStateMachine.wOccurredErrors;
}

/**
  * @brief It returns a 16 bit fields containing information about warnings
  *        currently present 
  * @param pHandle pointer of type  MotorStateMachineHandle_t.
  * @retval uint32_t  a 16 bit field that shoing occurred warning
  */
uint32_t MCStateMachine_GetWarningState()
{
  return MCStateMachine.wWarnings;
}

