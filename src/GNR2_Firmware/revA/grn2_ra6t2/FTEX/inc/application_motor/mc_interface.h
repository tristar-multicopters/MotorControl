/**
  * @file    mc_interface.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          MC Interface component. It allow controlling a motor and get measurements using a simple api.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_INTERFACE_H
#define __MC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_state_machine.h"
#include "speed_torq_ctrl.h"


/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MCI_BUFFER_EMPTY,                  /*!< If no buffered command has been
                                            called.*/
  MCI_COMMAND_NOT_ALREADY_EXECUTED,  /*!< If the buffered command condition
                                            hasn't already occurred.*/
  MCI_COMMAND_EXECUTED_SUCCESFULLY,  /*!< If the buffered command has been
                                            executed successfully.*/
  MCI_COMMAND_EXECUTED_UNSUCCESFULLY /*!< If the buffered command has been
                                            executed unsuccessfully.*/
} MCInterfaceCommandState_t ;

typedef enum
{
  MCI_NOCOMMANDSYET,        /*!< No command has been set by the user.*/
  MCI_EXECSPEEDRAMP,        /*!< ExecSpeedRamp command coming from the user.*/
  MCI_EXECTORQUERAMP,       /*!< ExecTorqueRamp command coming from the user.*/
  MCI_SETCURRENTREFERENCES, /*!< SetCurrentReferences command coming from the
                                 user.*/
} MCInterfaceUserCommands_t;

typedef struct
{
  MotorStateMachineHandle_t * pSTM; /*!< State machine object used by MCI.*/
  SpeednTorqCtrlHandle_t * pSpeedTorqCtrl; /*!< Speed and torque controller object used by MCI.*/
  pFOCVars_t pFOCVars;    /*!< Pointer to FOC vars used by MCI.*/
  MCInterfaceUserCommands_t LastCommand; /*!< Last command coming from the user.*/
  int16_t hFinalSpeed;        /*!< Final speed of last ExecSpeedRamp command.*/
  int16_t hFinalTorque;       /*!< Final torque of last ExecTorqueRamp
                                   command.*/
  qd_t Iqdref;     /*!< Current component of last
                                   SetCurrentReferences command.*/

  MCInterfaceCommandState_t CommandState; /*!< The status of the buffered command.*/
  STCModality_t LastModalitySetByUser; /*!< The last STCModality_t set by the
                                             user. */
} MotorControlInterfaceHandle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. It is also used to assign the
  *         state machine object, the speed and torque controller, and the FOC
  *         drive object to be used by MC Interface.
  * @param  pHandle pointer on the component instance to initialize.
  * @param  pSTM the state machine object used by the MCI.
  * @param  pSpeedTorqCtrl the speed and torque controller used by the MCI.
  * @param  pFOCVars pointer to FOC vars to be used by MCI.
  * @retval none.
  */
void MCInterface_Init(MotorControlInterfaceHandle_t * pHandle, MotorStateMachineHandle_t * pSTM, SpeednTorqCtrlHandle_t * pSpeedTorqCtrl, pFOCVars_t pFOCVars);

/**
  * @brief  This is usually a method managed by task. It must be called
  *         periodically in order to check the status of the related pSTM object
  *         and eventually to execute the buffered command if the condition
  *         occurs.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval none.
  */
void MCInterface_ExecBufferedCommands(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
  * @retval none.
  */
void MCInterface_ExecSpeedRamp(MotorControlInterfaceHandle_t * pHandle,  int16_t hFinalSpeed);

/**
  * @brief  This is a buffered command to set a motor torque ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  hFinalTorque is the value of motor torque reference at the end of
  *         the ramp. This value represents actually the Iq current expressed in
  *         digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.
  * @retval none.
  */
void MCInterface_ExecTorqueRamp(MotorControlInterfaceHandle_t * pHandle,  int16_t hFinalTorque);

/**
  * @brief  This is a buffered command to set directly the motor current
  *         references Iq and Id. This commands don't become active as soon as
  *         it is called but it will be executed when the pSTM state is
  *         START_RUN or RUN. User can check the status of the command calling
  *         the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  Iqdref current references on qd reference frame in qd_t
  *         format.
  * @retval none.
  */
void MCInterface_SetCurrentReferences(MotorControlInterfaceHandle_t * pHandle, qd_t Iqdref);

/**
  * @brief  This is a user command used to begin the start-up procedure.
  *         If the state machine is in IDLE state the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  *         Before calling MCI_StartMotor it is mandatory to execute one of
  *         these commands:\n
  *         MCI_ExecSpeedRamp\n
  *         MCI_ExecTorqueRamp\n
  *         MCI_SetCurrentReferences\n
  *         Otherwise the behaviour in run state will be unpredictable.\n
  *         <B>Note:</B> The MCI_StartMotor command is used just to begin the
  *         start-up procedure moving the state machine from IDLE state to
  *         IDLE_START. The command MCI_StartMotor is not blocking the execution
  *         of project until the motor is really running; to do this, the user
  *         have to check the state machine and verify that the RUN state (or
  *         any other state) has been reached.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_StartMotor(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  This is a user command used to begin the stop motor procedure.
  *         If the state machine is in RUN or START states the command is
  *         executed instantaneously otherwise the command is discarded. User
  *         must take care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MCI_StopMotor command is used just to begin the
  *         stop motor procedure moving the state machine to ANY_STOP.
  *         The command MCI_StopMotor is not blocking the execution of project
  *         until the motor is really stopped; to do this, the user have to
  *         check the state machine and verify that the IDLE state has been
  *         reached again.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_StopMotor(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  This is a user command used to indicate that the user has seen the
  *         error condition. If is possible, the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_FaultAcknowledged(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns information about the state of the last buffered command.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval CommandState_t  It can be one of the following codes:
  *         - MCI_BUFFER_EMPTY if no buffered command has been called.
  *         - MCI_COMMAND_NOT_ALREADY_EXECUTED if the buffered command
  *         condition hasn't already occurred.
  *         - MCI_COMMAND_EXECUTED_SUCCESFULLY if the buffered command has
  *         been executed successfully. In this case calling this function reset
  *         the command state to BC_BUFFER_EMPTY.
  *         - MCI_COMMAND_EXECUTED_UNSUCCESFULLY if the buffered command has
  *         been executed unsuccessfully. In this case calling this function
  *         reset the command state to BC_BUFFER_EMPTY.
  */
MCInterfaceCommandState_t  MCI_IsCommandAcknowledged(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval MotorState_t It returns the current state of the related pSTM object.
  */
MotorState_t  MCInterface_GetSTMState(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        historically occurred since the state machine has been moved into
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint16_t MCInterface_GetOccurredFaults(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about about currently
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint16_t MCInterface_GetCurrentFaults(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns the modality of the speed and torque controller.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval STCModality_t It returns the modality of STC. It can be one of
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STCModality_t MCInterface_GetControlMode(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns the motor direction imposed by the last command
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed,
  *         hFinalTorque or Iqdref.q of the last command.
  */
int16_t MCInterface_GetImposedMotorDirection(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns information about the last ramp final speed sent by the
  *         user expressed in tenths of HZ.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t last ramp final speed sent by the user expressed in tehts
  *         of HZ.
  */
int16_t MCInterface_GetLastRampFinalSpeed(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool MCInterface_IsRampCompleted(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  Stop the execution of ongoing ramp.
  * @param  pHandle Pointer on the component instance to work on.
  */
void MCInterface_StopRamp(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns speed sensor reliability with reference to the sensor
  *         actually used for reference frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the speed sensor utilized for reference
  *         frame transformation and (in speed control mode) for speed
  *         regulation is reliable, false otherwise
  */
bool MCInterface_GetSpdSensorReliability(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT and related to the sensor actually
  *         used by FOC algorithm
  * @param  pHandle Pointer on the component instance to work on.
  */
int16_t MCInterface_GetAvrgMecSpeedUnit(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in the unit defined by #SPEED_UNIT
  *
  * @param  pHandle Pointer on the component instance to work on.
  *
  */
int16_t MCInterface_GetMecSpeedRefUnit(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current Iab in ab_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval ab_t Stator current Iab
  */
ab_t MCInterface_GetIab(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current Ialphabeta in AlphaBeta_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval AlphaBeta_t Stator current Ialphabeta
  */
AlphaBeta_t MCInterface_GetIalphabeta(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current Iqd in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval qd_t Stator current Iqd
  */
qd_t MCInterface_GetIqd(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current IqdHF in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval qd_t Stator current IqdHF if HFI is selected as main
  *         sensor. Otherwise it returns { 0, 0}.
  */
qd_t MCInterface_GetIqdHF(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current Iqdref in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval qd_t Stator current Iqdref
  */
qd_t MCInterface_GetIqdref(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current Vqd in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval qd_t Stator current Vqd
  */
qd_t MCInterface_GetVqd(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns stator current Valphabeta in AlphaBeta_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval AlphaBeta_t Stator current Valphabeta
  */
AlphaBeta_t MCInterface_GetValphabeta(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns the rotor electrical angle actually used for reference
  *         frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Rotor electrical angle in dpp format
  */
int16_t MCInterface_GetElAngledpp(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Teref
  */
int16_t MCInterface_GetTeref(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used:
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MCInterface_GetPhaseCurrentAmplitude(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in
  *         s16V. To convert s16V into Volts following formula must be used:
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MCInterface_GetPhaseVoltageAmplitude(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  It re-initializes Iqdref variables with their default values.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval none
  */
void MCInterface_ClearIqdref(MotorControlInterfaceHandle_t * pHandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_INTERFACE_H */
