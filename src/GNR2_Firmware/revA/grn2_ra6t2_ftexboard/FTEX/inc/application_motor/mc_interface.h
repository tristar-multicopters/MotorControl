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
#include "flux_weakening_ctrl.h"
#include "mc_state_machine.h"
#include "speed_torq_ctrl.h"
#include "bus_voltage_sensor.h"
#include "r_divider_bus_voltage_sensor.h"
#include "gnr_parameters.h"
#include "Regen.h"

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initializes the parameters related to the battery (max power, max current,
  *         undervoltage threshold)
  * @param  MCSetup: VC parameters used to initialize the MC layer
  * @retval none.
  */
void MCInterface_PowerInit(MC_Setup_t MCSetup);

/**
  * @brief  Initializes the speed limit enable
  * @param  MCSetup: VC parameters used to initialize the MC layer
  * @retval none.
  */
void MCInterface_SpeedLimitEnInit(MC_Setup_t MCSetup);

/**
  * @brief  This is usually a method managed by task. It must be called
  *         periodically in order to check the status of the related pSTM object
  *         and eventually to execute the buffered command if the condition
  *         occurs.
  * @retval none.
  */
void MCInterface_ExecBufferedCommands();

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
  * @retval none.
  */
void MCInterface_ExecSpeedRamp(int16_t hFinalSpeed);

/**
  * @brief  This is a buffered command to set a motor torque ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  hFinalTorque is the value of motor torque reference at the end of
  *         the ramp in cNm (Nm/100).
  * @retval none.
  */
void MCInterface_ExecTorqueRamp(int16_t hFinalTorque);

/**
  * @brief  This is a buffered command to set directly the motor current
  *         references Iq and Id. This commands don't become active as soon as
  *         it is called but it will be executed when the pSTM state is
  *         START_RUN or RUN. User can check the status of the command calling
  *         the MCI_IsCommandAcknowledged method.
  * @param  Iqdref current references on qd reference frame in qd_t
  *         format.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.
  * @retval none.
  */
void MCInterface_SetCurrentReferences(qd_t Iqdref);

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
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_StartMotor();

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
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_StopMotor();

/**
  * @brief  This is a user command used to indicate that the user has seen the
  *         error condition. If is possible, the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_CriticalFaultAcknowledged();

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @retval MotorState_t It returns the current state of the related pSTM object.
  */
MotorState_t  MCInterface_GetSTMState();

/**
  * @brief It returns a 16 bit fields containing information about critical faults
  *        historically occurred since the state machine has been moved into
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @retval uint16_t  16 bit fields with information about the critical faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned critical fault codes are listed here \endlink
  */
uint32_t MCInterface_GetOccurredCriticalFaults();

/**
  * @brief It returns a 16 bit fields containing information about current errors.
  * @retval uint16_t  16 bit fields with information about the errors
  */
uint32_t MCInterface_GetCurrentErrors();

/**
  * @brief It returns a 16 bit fields containing information about occurred errors.
  * @retval uint16_t  16 bit fields with information about the errors
  */
uint32_t MCInterface_GetOccurredErrors();

/**
  * @brief It returns a 16 bit fields containing information about warnings.
  * @retval uint16_t  16 bit fields with information about the warnings
  */
uint32_t MCInterface_GetOccurredWarning();

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @retval uint16_t  16 bit fields with information about about currently
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint32_t MCInterface_GetCurrentCriticalFaults();

/**
  * @brief  It returns the modality of the speed and torque controller.
  * @retval STCModality_t It returns the modality of STC. It can be one of
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STCModality_t MCInterface_GetControlMode();

/**
  * @brief  It returns the motor direction imposed by the last command
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed,
  *         hFinalTorque or Iqdref.q of the last command.
  */
int16_t MCInterface_GetImposedMotorDirection();

/**
  * @brief  It returns information about the last ramp final speed sent by the
  *         user expressed in tenths of HZ.
  * @retval int16_t last ramp final speed sent by the user expressed in tehts
  *         of HZ.
  */
int16_t MCInterface_GetLastRampFinalSpeed();

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool MCInterface_IsRampCompleted();

/**
  * @brief  Stop the execution of ongoing ramp.
  */
void MCInterface_StopRamp();

/**
  * @brief  It returns speed sensor reliability with reference to the sensor
  *         actually used for reference frame transformation
  * @retval bool It returns true if the speed sensor utilized for reference
  *         frame transformation and (in speed control mode) for speed
  *         regulation is reliable, false otherwise
  */
bool MCInterface_GetSpdSensorReliability();

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT and related to the sensor actually
  *         used by FOC algorithm
  */
int16_t MCInterface_GetAvrgMecSpeedUnit();

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in the unit defined by #SPEED_UNIT
  */
int16_t MCInterface_GetMecSpeedRefUnit();

/**
  * @brief  It returns stator current Iab in ab_t format
  * @retval ab_t Stator current Iab
  */
ab_t MCInterface_GetIab();

/**
  * @brief  It returns stator current Ialphabeta in AlphaBeta_t format
  * @retval AlphaBeta_t Stator current Ialphabeta
  */
AlphaBeta_t MCInterface_GetIalphabeta();

/**
  * @brief  It returns stator current Iqd in qd_t format
  * @retval qd_t Stator current Iqd
  */
qd_t MCInterface_GetIqd();

/**
  * @brief  It returns stator current IqdHF in qd_t format
  * @retval qd_t Stator current IqdHF if HFI is selected as main
  *         sensor. Otherwise it returns { 0, 0}.
  */
qd_t MCInterface_GetIqdHF();

/**
  * @brief  It returns stator current Iqdref in qd_t format
  * @retval qd_t Stator current Iqdref
  */
qd_t MCInterface_GetIqdref();

/**
  * @brief  It returns stator current Vqd in qd_t format
  * @retval qd_t Stator current Vqd
  */
qd_t MCInterface_GetVqd();

/**
  * @brief  It returns stator current Valphabeta in AlphaBeta_t format
  * @retval AlphaBeta_t Stator current Valphabeta
  */
AlphaBeta_t MCInterface_GetValphabeta();

/**
  * @brief  It returns the rotor electrical angle actually used for reference
  *         frame transformation
  * @retval int16_t Rotor electrical angle in dpp format
  */
int16_t MCInterface_GetElAngledpp();

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @retval int16_t Teref
  */
int16_t MCInterface_GetTeref();

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used:
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MCInterface_GetPhaseCurrentAmplitude();

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in
  *         s16V. To convert s16V into Volts following formula must be used:
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MCInterface_GetPhaseVoltageAmplitude();

/**
  * @brief  Converts the digital voltage of the bus to a value in volts * 100
  *         Function has been added to enable the battery monitoring module in 
  *         vehicle control to have acces to the bus voltage.
  * @retval Value of the bus voltage in volts time 100 so 10v would be 1000
  */
uint16_t MCInterface_GetBusVoltageInVoltx100();

//desctribtion alter

/**
  * @brief  Getting the controller NTC temperature value
  * @retval Value of the heatsink temperature in celsius degree
  */
int16_t MCInterface_GetControllerTemp();

/**
  * @brief  Getting the Motor NTC temperature value
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Value of the motor temperature in celsius degree
  */
int16_t MCInterface_GetMotorTemp();

/**
  * @brief  Get the Max safe current from motor control.
  * @param  pHandle : handle of the MCI interface 
  * @retval the maximum current in amps
  */
int16_t MCInterface_GetMaxCurrent();

/**
  * @brief  Get the maximum ongoing current,
  * @param  pHandle : handle of the MCI interface
  * @retval the value in amps
  */
int16_t MCInterface_GetOngoingMaxCurrent();

/**
  * @brief  Set the maximum ongoing current, 
  * @retval nothing
  */
void MCInterface_SetOngoingMaxCurrent(int16_t aCurrent);

/**
  * @brief  update the wheelRPM in the speed position feedback handle 
  * @retval nothing
  */
void  MCInterface_SetWheelRPM(uint16_t aWheelRPM);

/** @brief  Get the current torq reference 
  * @retval nothing
  */
int16_t MCInterface_GetTorqueReference(uint8_t Motor);

/**
  * @brief  Get the max application power
  * @retval max app power
  */
uint16_t MCInterface_GetMaxPositivePower();


/**
  * @brief  Enable regen 
  * @retval enabling regenerative feature
  */
void MCInterface_EnableRegen();

/**
  * @brief  Disable regen 
  * @retval Disabling regenerative feature 
  */
void MCInterface_DisableRegen();

/**
  * @brief  Set the max negative allowed regen current in amps
  * @param minSpeed min speed regen applies
  * @return true if succsessful, false otherwise
  */
bool MCInterface_SetRegenMaxNegativeCurrent(int16_t Idc_Negative);

/**
  * @brief  Get the max negative regen current in amps
  * @return max negative regen current
  */
int16_t MCInterface_GetRegenMaxNegativeCurrent();

/**
  * @brief  Set the min negative regen current 
  * @param Idc_Negative min negative regen current
  * @return true if succsessful, false otherwise
  */
bool MCInterface_SetMinNegativeCurrent(int16_t MinCurrent);

/**
  * @brief  Get the min negative regen current
  * @return min negative regen current
  */
int16_t MCInterface_GetMinNegativeCurrent();

/**
  * @brief  Set the increasing rate of regen in percent per MC cycle
  * @return true if succsessful, false otherwise
  */
bool MCInterface_SetRegenRampPercent(uint16_t hRampPercent);

/**
  * @brief  Get the increasing rate of regen in percent per MC cycle
  * @return rate of regen value
  */
uint16_t MCInterface_GetRegenRampPercent();

/**
  * @brief  Set the increasing rate of max negative battery current in Nm per second
  * @param hMaxVoltage max negative voltage
  * @return true if succsessful, false otherwise
  */
bool MCInterface_SetRegenMaxVoltage(uint16_t hMaxVoltage);

/**
  * @brief  Get the increasing rate of max allowed voltage to do the regen
  * @return Max allowed voltage
  */
uint16_t MCInterface_GetRegenMaxVoltage();

/**
  * @brief Set the level of regen
  * @param level level of regen to be applied
  * @return true if succsessful, false otherwise
  */
bool MCInterface_SetRegenLevelPercent(uint8_t Level);


/**
  * @brief  Get the motor gear ratio
  * @retval motor gear ratio
  */
float MCInterface_GetMotorGearRatio();

/**
  * @brief  Get the motor type
  * @retval motor type
  */
MotorType_t MCInterface_GetMotorType();

/**
  * @brief  Get the nominal torque
  * @retval nominal torque
  */
uint16_t MCInterface_GetNominalTorque();

/**
  * @brief  Get the starting torque
  * @retval starting torque
  */
uint16_t MCInterface_GetStartingTorque();

/**
  * @brief  Get the RS value
  * @retval RS
  */
float MCInterface_GetRS();

/**
  * @brief  Get the number of magnets on the wheel speed sensor
  * @retval number of magnets on the wheel speed sensor
  */
uint8_t MCInterface_GetWheelSpdSensorNbrPerRotation();

/**
  * @brief  Get whether the motor temp sensor is mixed
  * @retval whether the motor temp sensor is mixed
  */
bool MCInterface_GetMotorTempSensorMixed();

/**
  * @brief  Enable flux weakening 
  * @retval enabling flux weakening feature
  */
void MCInterface_EnableFluxWeakening();

/**
  * @brief  Disable flux weakening 
  * @retval Disabling flux weakening feature
  */
void MCInterface_DisableFluxWeakening();

/**
  * @brief  Set speed limit of the wheel 
  * @param  Wheel Rpm
  */
void MCInterface_SetSpeedLimitWheelRpm(uint16_t wheelRpm);

/**
  * @brief  Set speed limit of the motor 
  * @param  Motor Rpm
  */
void MCInterface_SetSpeedLimit(int16_t desiredMotorRPM);

/**
  * @brief  Get max measurable current
  * @retval Max measurable current
  */
float MCInterface_GetMaxMeasurableCurrent();

/**
  * @brief  Set max bus current
  * @param  Max bus current
  */
void MCInterface_SetMaxBusCurrent(uint16_t maxBusCurrent);

/**
  * @brief  Set max continuous current
  * @param  Max continuous current
  */
void MCInterface_SetMaxContinuousCurrent(uint16_t maxContinuousCurrent);

/**
  * @brief  Set power foldback end value
  * @param  Power foldback end value
  */
void MCInterface_SetPowerFoldbackEndValue(int32_t endValue);

/**
  * @brief  Set power foldback range
  * @param  Power foldback range
  */
void MCInterface_SetPowerFoldbackRange(uint16_t range);

/**
  * @brief  Set motor temp sensor type
  * @param  Motor temp sensor type
  */
void MCInterface_SetMotorTempSensorType(uint8_t sensorType);

/**
  * @brief  Set motor NTC Beta coefficient
  * @param  Motor NTC Beta coefficient
  */
void MCInterface_SetMotorNTCBetaCoef(uint16_t NTCBetaCoef);

/**
  * @brief  Set motor NTC resistance coefficient
  * @param  Motor NTC resistance coefficient
  */
void MCInterface_SetMotorNTCResistanceCoef(float NTCResCoef);


/**
  * @brief  Set is motor signal mixed
  * @param  Is motor signal mixed
  */
void MCInterface_SetIsMotorSignalMixed(bool value);

/**
  * @brief  Set is min signal threshold value for mixed WSS and temp signals
  * @param  Min signal threshold value
  */
void MCInterface_SetMinSignalThresholdValueMixed(uint16_t value);

/**
  * @brief  Set max wheel speed period for mixed WSS and temp signals
  * @param  Max wheel speed period
  */
void MCInterface_SetMaxWheelSpeedPeriodUsValueMixed(uint32_t value);

/**
  * @brief  Get extracted wheel speed from mixed WSS and temp signals
  * @retval Wheel speed
  */
float MCInterface_GetExtractedWheelSpeedMixed(void);

#if AUTOTUNE_ENABLE
/**
  * @brief  This is a user command used to enter motor tuning mode.
  *         Commands to motor tuner (using the motor identification API) are only processed when motor is in this mode.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
  
bool MCInterface_StartMotorTuning();

/**
  * @brief  This is a user command used to exit motor tuning mode.
  *         Commands to motor tuner (using the motor identification API) are only processed when motor is in this mode.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCInterface_StopMotorTuning();
#endif

/**
  * @brief  This is a user command used to enter motor tuning mode.
  *         Commands to motor tuner (using the motor identification API) are only processed when motor is in this mode.
  */
void MCInterface_EnableRegen(void);

/**
  * @brief  Disable regen 
  */
void MCInterface_DisableRegen(void);

/**
  * @brief  Set the max negative battery current in amps
  * @param maxCurrent max negative current 
  */
bool MCInterface_SetRegenMaxCurrent(int16_t maxCurrent);

/**
  * @brief  Get the max negative battery current in amps
  */
int16_t MCInterface_GetRegenMaxCurrent(void);

/**
  * @brief  Get the max negative battery current in amps
  * @param minSpeed min speed regen applies
  */
bool MCInterface_SetRegenMinSpeed(int16_t minSpeed);

/**
  * @brief  Get the increasing rate of max negative battery current in Nm per second
  * @param resetSpeed Speed for reseting regen PIDs
  */
bool MCInterface_SetRegenResetSpeed(int16_t resetSpeed);


#ifdef __cplusplus

#endif /* __cpluplus */

#endif /* __MC_INTERFACE_H */
