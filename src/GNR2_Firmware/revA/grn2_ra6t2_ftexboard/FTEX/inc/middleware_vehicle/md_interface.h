/**
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control multiple motor drives.
    *                     M1 is the local drive, whereas M2, M3, M4, ... can be controlled externally using this interface.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MD_INTERFACE_H
#define __MD_INTERFACE_H

#include "mc_interface.h"
#include "slave_mc_interface.h"

#define MDI_PERCENT 100 // Used to apply a % based gain

/*
*  Structure used to hold all motor controller instances (M1, M2, M3, ...).
*/
typedef struct
{
    MotorControlInterfaceHandle_t * pMCI;
    SlaveMotorHandle_t * pSlaveM2;
} MultipleDriveInterfaceHandle_t;

/**
  * @brief  This function update the virtual motor habdle with the provided feedback.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  pMCI Pointer to the MotorControlInterface handle of M1 (local motor controller of this ganrunner device)
  * @param  pSlaveM2 Pointer to the SlaveMotorControlInterface handle of M2 (motor controller outside this ganrunner device)
  * @param  pBatteryPower to the BatteryPower handle
  * @retval none.
  */
void MDI_Init(MultipleDriveInterfaceHandle_t * pHandle, MotorControlInterfaceHandle_t * pMCI, SlaveMotorHandle_t * pSlaveM2, MC_Setup_t MCSetup);

/**
  * @brief  This function update the virtual motor handle with the provided feedback.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  bMotor Motor number. Must be higher than M2
    *    @param    pFeedback pointer to a virtual motor feedback structure. Data will be copied in pHandle.
  * @retval none.
  */
void MDI_UpdateVirtualMotorFeedback(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, SlaveMotorFeedback_t Feedback);

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MDI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
  * @param    bMotor is the target motor number
  * @retval none.
  */
void MDI_ExecSpeedRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalSpeed);

/**
  * @brief  This is a buffered command to set a motor torque ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MDI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  hFinalTorque is the value of motor torque reference at the end of
  *         the ramp in cNm (Nm/100).
  * @param    bMotor is the target motor number
  * @retval none.
  */
void MDI_ExecTorqueRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalTorque);

/**
  * @brief  This function gets the bus voltage in volts x100
  * @param  pHandle Pointer on the component instance to work on.
  * @retval The bus voltage in volts x 100.
  */
uint16_t MDI_GetBusVoltageInVoltx100(MotorControlInterfaceHandle_t * pHandle);

/**
  * @brief  This is a buffered command to set directly the motor current
  *         references Iq and Id. This commands don't become active as soon as
  *         it is called but it will be executed when the pSTM state is
  *         START_RUN or RUN. User can check the status of the command calling
  *         the MDI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  Iqdref current references on qd reference frame in qd_t
  *         format.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.
  * @param  bMotor is the target motor number
  * @retval none.
  */
void MDI_SetCurrentReferences(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, qd_t Iqdref);

/**
  * @brief  This is a user command used to begin the start-up procedure.
  *         If the state machine is in IDLE state the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  *         Before calling MDI_StartMotor it is mandatory to execute one of
  *         these commands:\n
  *         MDI_ExecSpeedRamp\n
  *         MDI_ExecTorqueRamp\n
  *         MDI_SetCurrentReferences\n
  *         Otherwise the behaviour in run state will be unpredictable.\n
  *         <B>Note:</B> The MDI_StartMotor command is used just to begin the
  *         start-up procedure moving the state machine from IDLE state to
  *         IDLE_START. The command MDI_StartMotor is not blocking the execution
  *         of project until the motor is really running; to do this, the user
  *         have to check the state machine and verify that the RUN state (or
  *         any other state) has been reached.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MDI_StartMotor(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  This is a user command used to begin the stop motor procedure.
  *         If the state machine is in RUN or START states the command is
  *         executed instantaneously otherwise the command is discarded. User
  *         must take care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MDI_StopMotor command is used just to begin the
  *         stop motor procedure moving the state machine to ANY_STOP.
  *         The command MDI_StopMotor is not blocking the execution of project
  *         until the motor is really stopped; to do this, the user have to
  *         check the state machine and verify that the IDLE state has been
  *         reached again.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MDI_StopMotor(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  This is a user command used to indicate that the user has seen the
  *         error condition. If is possible, the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MDI_CriticalFaultAcknowledged(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval MotorState_t It returns the current state of the related pSTM object.
  */
MotorState_t  MDI_GetSTMState(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        historically occurred since the state machine has been moved into
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval uint16_t  16 bit fields with information about the faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint32_t MDI_GetOccurredCriticalFaults(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval uint16_t  16 bit fields with information about about currently
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint32_t MDI_GetCurrentCriticalFaults(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief It returns a 16 bit fields containing information about errors
  *        that historically occurred.
  * @param pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval uint16_t  16 bit fields with information about about currently
  *         present errors.
  */
uint32_t MDI_GetOccurredErrors(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief It returns a 16 bit fields containing information about warning
  *        currently present.
  * @param pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval uint16_t  16 bit fields with information about about currently
  *         present warnings.
  */
uint32_t MDI_GetOccurredWarnings(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns the modality of the speed and torque controller.
  * @param  pHandle Pointer on the component instance to work on.
  * @param    bMotor is the target motor number
  * @retval STCModality_t It returns the modality of STC. It can be one of
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STCModality_t MDI_GetControlMode(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns the motor direction imposed by the last command
  *         (MDI_ExecSpeedRamp, MDI_ExecTorqueRamp or MDI_SetCurrentReferences).
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed,
  *         hFinalTorque or Iqdref.q of the last command.
  */
int16_t MDI_GetImposedMotorDirection(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns information about the last ramp final speed sent by the
  *         user expressed in tenths of HZ.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval int16_t last ramp final speed sent by the user expressed in tehts
  *         of HZ.
  */
int16_t MDI_GetLastRampFinalSpeed(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool MDI_IsRampCompleted(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Stop the execution of ongoing ramp.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  */
void MDI_StopRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns speed sensor reliability with reference to the sensor
  *         actually used for reference frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval bool It returns true if the speed sensor utilized for reference
  *         frame transformation and (in speed control mode) for speed
  *         regulation is reliable, false otherwise
  */
bool MDI_GetSpdSensorReliability(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT and related to the sensor actually
  *         used by FOC algorithm
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  */
int16_t MDI_GetAvrgMecSpeedUnit(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in the unit defined by #SPEED_UNIT
  *
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  *
  */
int16_t MDI_GetMecSpeedRefUnit(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current Iab in ab_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval ab_t Stator current Iab
  */
ab_t MDI_GetIab(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current Ialphabeta in AlphaBeta_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval AlphaBeta_t Stator current Ialphabeta
  */
AlphaBeta_t MDI_GetIalphabeta(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current Iqd in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval qd_t Stator current Iqd
  */
qd_t MDI_GetIqd(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current IqdHF in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval qd_t Stator current IqdHF if HFI is selected as main
  *         sensor. Otherwise it returns { 0, 0}.
  */
qd_t MDI_GetIqdHF(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current Iqdref in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval qd_t Stator current Iqdref
  */
qd_t MDI_GetIqdref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current Vqd in qd_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval qd_t Stator current Vqd
  */
qd_t MDI_GetVqd(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns stator current Valphabeta in AlphaBeta_t format
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval AlphaBeta_t Stator current Valphabeta
  */
AlphaBeta_t MDI_GetValphabeta(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns the rotor electrical angle actually used for reference
  *         frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval int16_t Rotor electrical angle in dpp format
  */
int16_t MDI_GetElAngledpp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval int16_t Teref
  */
int16_t MDI_GetTeref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used:
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MDI_GetPhaseCurrentAmplitude(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in
  *         s16V. To convert s16V into Volts following formula must be used:
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MDI_GetPhaseVoltageAmplitude(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Getting the controller NTC temperature value
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t temperature
  */
int16_t MDI_GetControllerTemp(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Getting the motor NTC temperature value
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t temperature
  */
int16_t MDI_GetMotorTemp(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  It re-initializes Iqdref variables with their default values.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval none
  */
void MDI_Clear_Iqdref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Obtain the motor torque referenece for a motor
  * @param  pHandle Pointer on the component instance to work on.
  * @param  bMotor is the target motor number
  * @retval none
  */
uint16_t MDI_GetMotorTorqueReference(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor);

/**
  * @brief  Obtain the max application power
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of the max app power
  */
uint16_t MDI_GetMaxPositivePower(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Function sets the torque control speed limit does the conversion from desired 
  *         wheel speed in kmH to motor rpm.Is also the wrapper for the function SpdTorqCtrl_SetSpeedLimit
  * @param  pHandle Pointer on the component instance to work on.
  * @param  speedKMH is the desired speed 
  * @param  speedKMH is the gain that ahs to be appllied in %
  * @retval none
  */
void MDI_SetTorqueSpeedLimit(MultipleDriveInterfaceHandle_t * pHandle, uint16_t speedKMH, uint16_t gain);

/**
  * @brief  Update the wheel RPM value
  * @param  pHandle .
  * @param  aWheelRPM
  * @retval none
  */
void MDI_SetWheelRPM(MultipleDriveInterfaceHandle_t * pHandle, uint16_t aWheelRPM);

/**
  * @brief  Get the motor gear ratio
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of the motor gear raio
  */
float MDI_GetMotorGearRatio(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get the motor type
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of the motor gear raio
  */
MotorType_t MDI_GetMotorType(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get the nominal torque
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of the nominal torque
  */
uint16_t MDI_GetNominalTorque(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get the starting torque
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of the starting torque
  */
uint16_t MDI_GetStartingTorque(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get the RS value
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of RS
  */
float MDI_GetRS(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get the number of magnets on the wheel speed sensor
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Number of magnets on the wheel speed
  */
uint8_t MDI_GetWheelSpdSensorNbrPerRotation(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get the percentage of time that the wheel speed sensor spends on each magnet
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Percentage of time that the wheel speed sensor spends on each magnet
  */
float MDI_GetMotorWSSTimeOnOneMagnetPercent(MultipleDriveInterfaceHandle_t * pHandle);

/**
  * @brief  Get whether the motor temp sensor is mixed
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Whether the motor temp sensor is mixed
  */
bool MDI_GetMotorTempSensorMixed(MultipleDriveInterfaceHandle_t * pHandle);

#endif /* __MD_INTERFACE_H */
