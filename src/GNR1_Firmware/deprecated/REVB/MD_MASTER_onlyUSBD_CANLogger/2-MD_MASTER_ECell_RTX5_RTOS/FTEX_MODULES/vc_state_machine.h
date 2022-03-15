/**
  ******************************************************************************
  * @file    vc_state_machine.h
  * @author  Sami Bouzid, FTEX
  * @brief   This file contains all definitions and functions prototypes for the
  *          Vehicle Control State Machine component.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_STATEMACHINE_H
#define __VC_STATEMACHINE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/** @name Vehicle Fault source error codes */
/** @{ */
#define  VC_NO_ERROR   								(uint16_t)(0x0000u)      /**< @brief No error.*/
#define  VC_NO_FAULTS  								(uint16_t)(0x0000u)     /**< @brief No error.*/
#define  VC_M1_FAULTS  								(uint16_t)(0x0001u)  /**< @brief Error: Fault with motor 1.*/
#define  VC_M2_FAULTS  								(uint16_t)(0x0002u)     /**< @brief Error: Fault with motor 2.*/
#define	 VC_SW_ERROR   								(uint16_t)(0x0004u)     /**< @brief Error: Vehicle software error.*/
#define	 VC_MC_COMM_ERROR   				 	(uint16_t)(0x0008u)     /**< @brief Error: Communication with motor controller.*/
#define	 VC_M1_UNEXPECTED_BEHAVIOR   	(uint16_t)(0x0020u)     /**< @brief Error: Unexpected behavior of M1.*/
#define	 VC_M2_UNEXPECTED_BEHAVIOR   	(uint16_t)(0x0040u)     /**< @brief Error: Unexpected behavior of M2.*/
#define  VC_CAN_QUEUE_FULL						(uint16_t)(0x0060u)     /**< @brief Error: CAN Message queue is full.*/
#define	 VC_STARTUP_TIMEOUT				   	(uint16_t)(0x0080u)     /**< @brief Error: Startup procedure reached timeout.*/
/** @} */

/**
  * @brief  VC_State_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  V_IDLE = 0,             /*!< Motor controllers are off */
	V_STANDBY = 1,             /*!< Motor controllers are active but no torque applied */
	V_STANDBY_START = 8,		/*!< Pass-throught state, between standby and start  */
	V_START_FORWARD = 2,		/*!< Startup procedure in forward direction (mainly for sensorless drive)  */
  V_RUN_FORWARD = 6,      /*!< Torque applied to motor controllers in forward direction.  */
	V_START_BACKWARD = 3,		/*!< Startup procedure in backward direction (mainly for sensorless drive)  */
  V_RUN_BACKWARD = 7,     /*!< Torque applied to motor controllers in backward direction.  */
  V_FAULT_NOW = 10,       /*!< Persistent state, the state machine can be moved from
                           any condition directly to this state by
                           VCSTM_FaultProcessing method. This method also manage
                           the passage to the only allowed following state that
                           is V_FAULT_OVER */
  V_FAULT_OVER = 11,       /*!< Persistent state where the application is intended to
                          stay when the fault conditions disappeared. State machine is moved as
                          soon as the user has acknowledged the fault condition.
                      */
} VC_State_t;

/**
  * @brief  StateMachine class members definition
  */
typedef struct
{
  VC_State_t   bVState;          /*!< Variable containing state machine current
                                    state of vehicle */
  uint16_t  hVFaultNow;       /*!< Bit fields variable containing vehicle faults
                                    currently present */
  uint16_t  hVFaultOccurred;  /*!< Bit fields variable containing vehicle faults
                                    historically occurred since the state
                                    machine has been moved to FAULT_NOW state */
} VCSTM_Handle_t;


/*  Initializes all the component variables. */
void VCSTM_Init( VCSTM_Handle_t * pHandle );

/* It submits the request for moving the state machine into the state bVState */
bool VCSTM_NextState( VCSTM_Handle_t * pHandle, VC_State_t bVState );

/* It clocks both HW and SW faults processing and update the state machine accordingly */
VC_State_t VCSTM_FaultProcessing( VCSTM_Handle_t * pHandle, uint16_t hSetErrors, uint16_t
                             hResetErrors );

/* Returns the current state machine state */
VC_State_t VCSTM_GetState( VCSTM_Handle_t * pHandle );

/**
  * It reports to the state machine that the fault state has been
  *        acknowledged by the user.
  */
bool VCSTM_FaultAcknowledged( VCSTM_Handle_t * pHandle );

/**
  * It returns two 16 bit fields containing information about both faults
  * currently present and faults historically occurred since the state
  * machine has been moved into state
  */
uint32_t VCSTM_GetFaultState( VCSTM_Handle_t * pHandle );


#endif /* __VC_STATEMACHINE_H */


