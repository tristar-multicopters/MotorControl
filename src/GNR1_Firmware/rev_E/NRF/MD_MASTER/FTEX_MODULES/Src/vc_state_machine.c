/**
  ******************************************************************************
  * @file    vc_state_machine.c
  * @author  Sami Bouzid, FTEX
  * @brief   This file contains all definitions and functions prototypes for the
  *          Vehicle Control State Machine component.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "vc_state_machine.h"

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param pHandle pointer on the component instance to initialize.
  * @retval none.
  */
void VCSTM_Init( VCSTM_Handle_t * pHandle )
{

  pHandle->bVState = V_IDLE;
  pHandle->hVFaultNow = VC_NO_FAULTS;
  pHandle->hVFaultOccurred = VC_NO_FAULTS;
}

/**
  * @brief It submits the request for moving the state machine into the state
  *        specified by bState (V_FAULT_NOW and V_FAUL_OVER are not handled by this
  *        method). Accordingly with the current state, the command is really
  *        executed (state machine set to bState) or discarded (no state
  *        changes).
  *        If requested state can't be reached the return value is false and the
  *        VC_SW_ERROR is raised.
  * @param pHanlde pointer of type  VCSTM_Handle_t.
  * @param bState New requested state
  * @retval bool It returns true if the state has been really set equal to
  *         bState, false if the requested state can't be reached
  */
bool VCSTM_NextState( VCSTM_Handle_t * pHandle, VC_State_t bState )
{
  bool bChangeState = false;
  VC_State_t bCurrentState = pHandle->bVState;
  VC_State_t bNewState = bCurrentState;

  switch ( bCurrentState )
  {
    case V_IDLE:
      if ( bState == V_STANDBY )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case V_STANDBY:
      if ( bState == V_STANDBY_START )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
		case V_STANDBY_START:
      if ( bState == V_START )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    case V_START:
      if ( bState == V_RUN || bState == V_ANY_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case V_RUN:
      if ( bState == V_ANY_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    case V_ANY_STOP:
      if ( bState == V_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
			
    case V_STOP:
      if ( bState == V_IDLE )
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
    pHandle->bVState = bNewState;
  }
  else
  {
		/* Raise a software error */
		VCSTM_FaultProcessing( pHandle, VC_SW_ERROR, 0u );
  }

  return ( bChangeState );
}

/**
  * @brief It clocks both HW and SW faults processing and update the state
  *        machine accordingly with hSetErrors, hResetErrors and present state.
  *        Refer to State_t description for more information about fault states.
  * @param pHanlde pointer of type  VCSTM_Handle_t
  * @param hSetErrors Bit field reporting faults currently present
  * @param hResetErrors Bit field reporting faults to be cleared
  * @retval VC_State_t New state machine state after fault processing
  */
VC_State_t VCSTM_FaultProcessing( VCSTM_Handle_t * pHandle, uint16_t hSetErrors, uint16_t
                             hResetErrors )
{
  VC_State_t LocalState =  pHandle->bVState;

  /* Set current errors */
  pHandle->hVFaultNow = ( pHandle->hVFaultNow | hSetErrors ) & ( ~hResetErrors );
  pHandle->hVFaultOccurred |= hSetErrors;

  if ( LocalState == V_FAULT_NOW )
  {
    if ( pHandle->hVFaultNow == VC_NO_FAULTS )
    {
      pHandle->bVState = V_FAULT_OVER;
      LocalState = V_FAULT_OVER;
    }
  }
  else
  {
    if ( pHandle->hVFaultNow != VC_NO_FAULTS )
    {
      pHandle->bVState = V_FAULT_NOW;
      LocalState = V_FAULT_NOW;
    }
  }

  return ( LocalState );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Returns the current state machine state
  * @param  pHanlde pointer of type  VCSTM_Handle_t
  * @retval VC_State_t Current state machine state
  */
VC_State_t VCSTM_GetState( VCSTM_Handle_t * pHandle )
{
  return ( pHandle->bVState );
}


/**
  * @brief It reports to the state machine that the fault state has been
  *        acknowledged by the user.
  * @param pHanlde pointer of type  VCSTM_Handle_t
  * @retval bool true if the state machine has been moved to V_IDLE, false if the
  *        method call had no effects
  */
bool VCSTM_FaultAcknowledged( VCSTM_Handle_t * pHandle )
{
  bool bToBeReturned = false;

  if ( pHandle->bVState == V_FAULT_OVER )
  {
    pHandle->bVState = V_IDLE;
    pHandle->hVFaultOccurred = VC_NO_FAULTS;
    bToBeReturned = true;
  }

  return ( bToBeReturned );
}


/**
  * @brief It returns two 16 bit fields containing information about both faults
  *        currently present and faults historically occurred since the state
  *        machine has been moved into state
  * @param pHanlde pointer of type  VCSTM_Handle_t.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present faults. In the least
  *         significant half are stored the information about the faults
  *         historically occurred since the state machine has been moved into
  *         V_FAULT_NOW state
  */
uint32_t VCSTM_GetFaultState( VCSTM_Handle_t * pHandle )
{
  uint32_t LocalFaultState;

  LocalFaultState = ( uint32_t )( pHandle->hVFaultOccurred );
  LocalFaultState |= ( uint32_t )( pHandle->hVFaultNow ) << 16;

  return LocalFaultState;
}

