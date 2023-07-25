/**
  * @file    mc_state_machine.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Motor Control State Machine component of the Motor Control application.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_STATEMACHINE_H
#define __MC_STATEMACHINE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mc_type.h"


/* Exported types ------------------------------------------------------------*/
/**
  * @brief  MotorState_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  M_ICLWAIT = 12,         /*!< Persistent state, the system is waiting for ICL
                           deactivation. Is not possible to run the motor if
                           ICL is active. Until the ICL is active the state is
                           forced to ICLWAIT, when ICL become inactive the state
                           is moved to IDLE */
  M_IDLE = 0,             /*!< Persistent state, following state can be IDLE_START
                           if a start motor command has been given or
                           IDLE_ALIGNMENT if a start alignment command has been
                           given */
  M_IDLE_ALIGNMENT = 1,   /*!< "Pass-through" state containg the code to be executed
                           only once after encoder alignment command.
                           Next states can be ALIGN_CHARGE_BOOT_CAP or
                           ALIGN_OFFSET_CALIB according the configuration. It
                           can also be ANY_STOP if a stop motor command has been
                           given. */
  M_ALIGN_CHARGE_BOOT_CAP = 13,/*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           ALIGN_OFFSET_CALIB. It can also be ANY_STOP if a stop
                           motor command has been given. */
  M_ALIGN_OFFSET_CALIB = 14,/*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           ALIGN_CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  M_ALIGN_CLEAR = 15,     /*!< "Pass-through" state in which object is cleared and
                           set for the startup.
                           Next state will be ALIGNMENT. It can also be ANY_STOP
                           if a stop motor command has been given. */
  M_ALIGNMENT = 2,        /*!< Persistent state in which the encoder are properly
                           aligned to set mechanical angle, following state can
                           only be ANY_STOP */
  M_IDLE_START = 3,       /*!< "Pass-through" state containg the code to be executed
                           only once after start motor command.
                           Next states can be CHARGE_BOOT_CAP or OFFSET_CALIB
                           according the configuration. It can also be ANY_STOP
                           if a stop motor command has been given. */
  M_CHARGE_BOOT_CAP = 16, /*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           OFFSET_CALIB. It can also be ANY_STOP if a stop motor
                           command has been given. */
  M_OFFSET_CALIB = 17,    /*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  M_CLEAR = 18,           /*!< "Pass-through" state in which object is cleared and
                           set for the startup.
                           Next state will be START. It can also be ANY_STOP if
                           a stop motor command has been given. */
  M_START = 4,            /*!< Persistent state where the motor start-up is intended
                           to be executed. The following state is normally
                           SWITCH_OVER or RUN as soon as first validated speed is
                           detected. Another possible following state is
                           ANY_STOP if a stop motor command has been executed */
  M_SWITCH_OVER = 19,     /**< TBD */
  M_START_RUN = 5,        /*!< "Pass-through" state, the code to be executed only
                           once between START and RUN states itâ€™s intended to be
                           here executed. Following state is normally  RUN but
                           it can also be ANY_STOP  if a stop motor command has
                           been given */
  M_RUN = 6,              /*!< Persistent state with running motor. The following
                           state is normally ANY_STOP when a stop motor command
                           has been executed */
  M_ANY_STOP = 7,         /*!< "Pass-through" state, the code to be executed only
                           once between any state and STOP itâ€™s intended to be
                           here executed. Following state is normally STOP */
  M_STOP = 8,             /*!< Persistent state. Following state is normally
                           STOP_IDLE as soon as conditions for moving state
                           machine are detected */
  M_STOP_IDLE = 9,        /*!< "Pass-through" state, the code to be executed only
                           once between STOP and IDLE itâ€™s intended to be here
                           executed. Following state is normally IDLE */
  M_FAULT_NOW = 10,       /*!< Persistent state, the state machine can be moved from
                           any condition directly to this state by
                           MCStateMachine_FaultProcessing method. This method also manage
                           the passage to the only allowed following state that
                           is FAULT_OVER */
  M_FAULT_OVER = 11,       /*!< Persistent state where the application is intended to
                          stay when the fault conditions disappeared. Following
                          state is normally STOP_IDLE, state machine is moved as
                          soon as the user has acknowledged the fault condition.
                      */
  M_WAIT_STOP_MOTOR = 20

} MotorState_t;

/**
  * @brief  StateMachine class members definition
  */
typedef struct
{
  MotorState_t   bState;     /*!< Variable containing state machine current
                                    state */
  uint16_t  hFaultNow;       /*!< Bit fields variable containing faults
                                    currently present */
  uint16_t  hFaultOccurred;  /*!< Bit fields variable containing faults
                                    historically occurred since the state
                                    machine has been moved to FAULT_NOW state */
  uint16_t  hWarnings;        /*!< containing warning that raised by MC Layer */
} MotorStateMachineHandle_t;



/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param pHandle pointer on the component instance to initialize.
  * @retval none.
  */
void MCStateMachine_Init(MotorStateMachineHandle_t * pHandle);

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
  * @param pHanlde pointer of type  MotorStateMachineHandle_t.
  * @param bState New requested state
  * @retval bool It returns true if the state has been really set equal to
  *         bState, false if the requested state can't be reached
  */
bool MCStateMachine_NextState(MotorStateMachineHandle_t * pHandle, MotorState_t bState);

/**
  * @brief It clocks both HW and SW faults processing and update the state
  *        machine accordingly with hSetErrors, hResetErrors and present state.
  *        Refer to MotorState_t description for more information about fault states.
  * @param pHanlde pointer of type  MotorStateMachineHandle_t
  * @param hSetErrors Bit field reporting faults currently present
  * @param hResetErrors Bit field reporting faults to be cleared
  * @retval MotorState_t New state machine state after fault processing
  */
MotorState_t MCStateMachine_FaultProcessing(MotorStateMachineHandle_t * pHandle, uint16_t hSetErrors, uint16_t
                             hResetErrors);

/**
  * @brief It clocks both HW and SW warning processing
  *        machine accordingly with hSetWarnings, hResetWarnings and present state.
  *        Refer to MotorState_t description for more information about fault states.
  * @param pHanlde pointer of type  MotorStateMachineHandle_t
  * @param hSetWarnings Bit field reporting warnings currently present
  * @param hResetWarnings Bit field reporting warnings to be cleared
  * @retval none.
  */
void MCStateMachine_WarningHandling(MotorStateMachineHandle_t * pHandle, uint16_t hSetWarnings, uint16_t  hResetWarnings);

/**
  * @brief  Returns the current state machine state
  * @param  pHanlde pointer of type  MotorStateMachineHandle_t
  * @retval MotorState_t Current state machine state
  */
MotorState_t MCStateMachine_GetState(MotorStateMachineHandle_t * pHandle);

/**
  * @brief It reports to the state machine that the fault state has been
  *        acknowledged by the user. If the state machine is in FAULT_OVER state
  *        then it is moved into STOP_IDLE and the bit field variable containing
  *        information about the faults historically occured is cleared.
  *        The method call is discarded if the state machine is not in FAULT_OVER
  * @param pHanlde pointer of type  MotorStateMachineHandle_t
  * @retval bool true if the state machine has been moved to IDLE, false if the
  *        method call had no effects
  */
bool MCStateMachine_FaultAcknowledged(MotorStateMachineHandle_t * pHandle);

/**
  * @brief It returns two 16 bit fields containing information about both faults
  *        currently present and faults historically occurred since the state
  *        machine has been moved into state
  * @param pHanlde pointer of type  MotorStateMachineHandle_t.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present faults. In the least
  *         significant half are stored the information about the faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state
  */
uint32_t MCStateMachine_GetFaultState(MotorStateMachineHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about warnings
  *        currently present 
  * @param pHanlde pointer of type  MotorStateMachineHandle_t.
  * @retval uint32_t  a 16 bit field that shoing occured warning
  */
uint32_t MCStateMachine_GetWarningState(MotorStateMachineHandle_t * pHandle);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_STATEMACHINE_H */
