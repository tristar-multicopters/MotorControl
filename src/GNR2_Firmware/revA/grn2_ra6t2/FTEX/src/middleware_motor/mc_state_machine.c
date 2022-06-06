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


void MCStateMachine_Init( MotorStateMachineHandle_t * pHandle )
{

  pHandle->bState = M_IDLE;
  pHandle->hFaultNow = MC_NO_FAULTS;
  pHandle->hFaultOccurred = MC_NO_FAULTS;
}


bool MCStateMachine_NextState( MotorStateMachineHandle_t * pHandle, MotorState_t bState )
{
  bool bChangeState = false;
  MotorState_t bCurrentState = pHandle->bState;
  MotorState_t bNewState = bCurrentState;

  switch ( bCurrentState )
  {
    case M_ICLWAIT:
      if ( bState == M_IDLE )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
    case M_IDLE:
      if ( ( bState == M_IDLE_START ) || ( bState == M_IDLE_ALIGNMENT )
           || ( bState == M_ICLWAIT ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_IDLE_ALIGNMENT:
      if ( ( bState == M_ANY_STOP ) || ( bState == M_ALIGN_CHARGE_BOOT_CAP )
           || ( bState == M_ALIGN_OFFSET_CALIB ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGN_CHARGE_BOOT_CAP:
      if ( ( bState == M_ALIGN_OFFSET_CALIB ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGN_OFFSET_CALIB:
      if ( ( bState == M_ALIGN_CLEAR ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGN_CLEAR:
      if ( ( bState == M_ALIGNMENT ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ALIGNMENT:
      if ( bState == M_ANY_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_IDLE_START:
      if ( ( bState == M_ANY_STOP ) || ( bState == M_CHARGE_BOOT_CAP ) ||
           ( bState == M_START ) ||
           ( bState == M_OFFSET_CALIB ) || ( bState == M_IDLE_ALIGNMENT ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_CHARGE_BOOT_CAP:
      if ( ( bState == M_OFFSET_CALIB ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_OFFSET_CALIB:
      if ( ( bState == M_CLEAR ) || ( bState == M_ANY_STOP ) || ( bState == M_WAIT_STOP_MOTOR ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

     case M_WAIT_STOP_MOTOR:
      if ( ( bState == M_CLEAR ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_CLEAR:
      if ( ( bState == M_START ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_START:
      if ( ( bState == M_SWITCH_OVER ) || ( bState == M_ANY_STOP ) || (bState == M_START_RUN) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_SWITCH_OVER:
      if ( ( bState == M_START ) || ( bState == M_ANY_STOP ) || (bState == M_START_RUN) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_START_RUN:
      if ( ( bState == M_RUN ) || ( bState == M_ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_RUN:
      if ( bState == M_ANY_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_ANY_STOP:
      if ( bState == M_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_STOP:
      if ( bState == M_STOP_IDLE )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case M_STOP_IDLE:
      if ( ( bState == M_IDLE ) || ( bState == M_ICLWAIT ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
    default:
      break;
  }

  if ( bChangeState )
  {
    pHandle->bState = bNewState;
  }
  else
  {
    if ( !( ( bState == M_IDLE_START ) || ( bState == M_IDLE_ALIGNMENT )
            || ( bState == M_ANY_STOP ) ) )
    {
      /* If new state is not a user command START/STOP raise a software error */
      MCStateMachine_FaultProcessing( pHandle, MC_SW_ERROR, 0u );
    }
  }

  return ( bChangeState );
}


MotorState_t MCStateMachine_FaultProcessing( MotorStateMachineHandle_t * pHandle, uint16_t hSetErrors, uint16_t
                             hResetErrors )
{
  MotorState_t LocalState =  pHandle->bState;

  /* Set current errors */
  pHandle->hFaultNow = ( pHandle->hFaultNow | hSetErrors ) & ( ~hResetErrors );
  pHandle->hFaultOccurred |= hSetErrors;

  if ( LocalState == M_FAULT_NOW )
  {
    if ( pHandle->hFaultNow == MC_NO_FAULTS )
    {
      pHandle->bState = M_FAULT_OVER;
      LocalState = M_FAULT_OVER;
    }
  }
  else
  {
    if ( pHandle->hFaultNow != MC_NO_FAULTS )
    {
      pHandle->bState = M_FAULT_NOW;
      LocalState = M_FAULT_NOW;
    }
  }

  return ( LocalState );
}


MotorState_t MCStateMachine_GetState( MotorStateMachineHandle_t * pHandle )
{
  return ( pHandle->bState );
}



bool MCStateMachine_FaultAcknowledged( MotorStateMachineHandle_t * pHandle )
{
  bool bToBeReturned = false;

  if ( pHandle->bState == M_FAULT_OVER )
  {
    pHandle->bState = M_STOP_IDLE;
    pHandle->hFaultOccurred = MC_NO_FAULTS;
    bToBeReturned = true;
  }

  return ( bToBeReturned );
}



uint32_t MCStateMachine_GetFaultState( MotorStateMachineHandle_t * pHandle )
{
  uint32_t LocalFaultState;

  LocalFaultState = ( uint32_t )( pHandle->hFaultOccurred );
  LocalFaultState |= ( uint32_t )( pHandle->hFaultNow ) << 16;

  return LocalFaultState;
}
