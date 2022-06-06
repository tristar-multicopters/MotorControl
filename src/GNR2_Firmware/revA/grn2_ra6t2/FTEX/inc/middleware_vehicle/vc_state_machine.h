/**
  * @file    vc_state_machine.h
  * @author  Sami Bouzid, FTEX
  * @brief   This file contains all definitions and functions prototypes for the
  *          Vehicle Control State Machine component.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_STATEMACHINE_H
#define __VC_STATEMACHINE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "vc_defines.h"

/**
  * @brief  VC_State_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  V_IDLE = 0,             	/*!< Vehicle is off */
	V_STANDBY = 1,            /*!< Vehicle is waiting for a start condition */
	V_STANDBY_START = 2,			/*!< Pass-throught state, between standby and start  */
	V_START = 3,							/*!< Startup procedure (mainly for sensorless drive)  */
  V_RUN = 4,      					/*!< Torque is applied to motors  */
	V_ANY_STOP = 5,						/*!< Pass-throught state, before stop  */
	V_STOP = 6,								/*!< Stop motors  */
  V_FAULT_NOW = 7,       	/*!< Persistent state, the state machine can be moved from
																	any condition directly to this state by
																	VCSTM_FaultProcessing method. This method also manage
																	the passage to the only allowed following state that
																	is V_FAULT_OVER */
  V_FAULT_OVER = 8,       	/*!< Persistent state where the application is intended to
																	stay when the fault conditions disappeared. State machine is moved as
																	soon as the user has acknowledged the fault condition.*/
} VC_State_t;

/**
  * @brief  StateMachine class members definition
  */
typedef struct
{
  VC_State_t bVState;          /*!< Variable containing state machine current
                                    state of vehicle */
  uint16_t hVFaultNow;       /*!< Bit fields variable containing vehicle faults
                                    currently present */
  uint16_t hVFaultOccurred;  /*!< Bit fields variable containing vehicle faults
                                    historically occurred since the state
                                    machine has been moved to FAULT_NOW state */
} VCSTM_Handle_t;


/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param pHandle pointer on the component instance to initialize.
  * @retval none.
  */
void VCSTM_Init( VCSTM_Handle_t * pHandle );

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
bool VCSTM_NextState( VCSTM_Handle_t * pHandle, VC_State_t bVState );

/**
  * @brief It clocks both HW and SW faults processing and update the state
  *        machine accordingly with hSetErrors, hResetErrors and present state.
  *        Refer to MotorState_t description for more information about fault states.
  * @param pHanlde pointer of type  VCSTM_Handle_t
  * @param hSetErrors Bit field reporting faults currently present
  * @param hResetErrors Bit field reporting faults to be cleared
  * @retval VC_State_t New state machine state after fault processing
  */
VC_State_t VCSTM_FaultProcessing( VCSTM_Handle_t * pHandle, uint16_t hSetErrors, uint16_t
                             hResetErrors );

/**
  * @brief  Returns the current state machine state
  * @param  pHanlde pointer of type  VCSTM_Handle_t
  * @retval VC_State_t Current state machine state
  */
VC_State_t VCSTM_GetState( VCSTM_Handle_t * pHandle );

/*	Function to get the current fault (if there's any) of the vehicle*/
uint16_t VCSTM_getCurrentFaults( VCSTM_Handle_t * pHandle );

/*	Function to get faults that occured of the vehicle	*/
uint16_t VCSTM_getOccurredFaults( VCSTM_Handle_t * pHandle );

/**
  * @brief It reports to the state machine that the fault state has been
  *        acknowledged by the user.
  * @param pHanlde pointer of type  VCSTM_Handle_t
  * @retval bool true if the state machine has been moved to V_IDLE, false if the
  *        method call had no effects
  */
bool VCSTM_FaultAcknowledged( VCSTM_Handle_t * pHandle );

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
uint32_t VCSTM_GetFaultState( VCSTM_Handle_t * pHandle );


#endif /* __VC_STATEMACHINE_H */


