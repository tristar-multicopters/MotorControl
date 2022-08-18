/**
  * @file    vc_state_machine.c
  * @author  Sami Bouzid, FTEX
  * @brief   This file contains all definitions and functions prototypes for the
  *          Vehicle Control State Machine component.
  */

/* Includes ------------------------------------------------------------------*/
#include "vc_state_machine.h"


void VCSTM_Init(VCSTM_Handle_t * pHandle)
{

  pHandle->bVState = V_IDLE;
  pHandle->hVFaultNow = VC_NO_FAULTS;
  pHandle->hVFaultOccurred = VC_NO_FAULTS;
}


bool VCSTM_NextState(VCSTM_Handle_t * pHandle, VC_State_t bState)
{
  bool bChangeState = false;
  VC_State_t bCurrentState = pHandle->bVState;
  VC_State_t bNewState = bCurrentState;

  switch (bCurrentState)
  {
    case V_IDLE:
      if (bState == V_STANDBY)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case V_STANDBY:
      if (bState == V_STANDBY_START)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
		case V_STANDBY_START:
      if (bState == V_START)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    case V_START:
      if (bState == V_RUN || bState == V_ANY_STOP)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case V_RUN:
      if (bState == V_ANY_STOP)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    case V_ANY_STOP:
      if (bState == V_STOP)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    case V_STOP:
      if (bState == V_IDLE)
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    default:
      break;
  }

  if (bChangeState)
  {
    pHandle->bVState = bNewState;
  }
  else
  {
		/* Raise a software error */
		VCSTM_FaultProcessing(pHandle, VC_SW_ERROR, 0u);
  }

  return (bChangeState);
}


VC_State_t VCSTM_FaultProcessing(VCSTM_Handle_t * pHandle, uint16_t hSetErrors, uint16_t
                             hResetErrors)
{
  VC_State_t LocalState =  pHandle->bVState;

  /* Set current errors */
  pHandle->hVFaultNow = (pHandle->hVFaultNow | hSetErrors) & (~hResetErrors);
  pHandle->hVFaultOccurred |= hSetErrors;

  if (LocalState == V_FAULT_NOW)
  {
    if (pHandle->hVFaultNow == VC_NO_FAULTS)
    {
      pHandle->bVState = V_FAULT_OVER;
      LocalState = V_FAULT_OVER;
    }
  }
  else
  {
    if (pHandle->hVFaultNow != VC_NO_FAULTS)
    {
      pHandle->bVState = V_FAULT_NOW;
      LocalState = V_FAULT_NOW;
    }
  }

  return (LocalState);
}


VC_State_t VCSTM_GetState(VCSTM_Handle_t * pHandle)
{
  return (pHandle->bVState);
}


bool VCSTM_FaultAcknowledged(VCSTM_Handle_t * pHandle)
{
  bool bToBeReturned = false;

  if (pHandle->bVState == V_FAULT_OVER)
  {
    pHandle->bVState = V_IDLE;
    pHandle->hVFaultOccurred = VC_NO_FAULTS;
    bToBeReturned = true;
  }

  return (bToBeReturned);
}


uint32_t VCSTM_GetFaultState(VCSTM_Handle_t * pHandle)
{
  uint32_t LocalFaultState;

  LocalFaultState = (uint32_t)(pHandle->hVFaultOccurred);
  LocalFaultState |= (uint32_t)(pHandle->hVFaultNow) << 16;

  return LocalFaultState;
}

