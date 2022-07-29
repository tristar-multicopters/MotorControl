/**
  * @file    speed_torq_ctrl.h
  * @author  Sami Bouzid, FTEX inc
  * @brief   This file contains all definitions and functions prototypes for the
  *          Speed & Torque Control component of the Motor Control application.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEEDNTORQCTRLCLASS_H
#define __SPEEDNTORQCTRLCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "speed_pos_fdbk.h"
#include "ramp_mngr.h"
#include "foldback.h"
#include "ntc_temperature_sensor.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Speed & Torque Control parameters definition
  */
typedef struct
{
    RampMngr_Handle_t TorqueRampMngr;
    RampMngr_Handle_t SpeedRampMngr;

    Foldback_Handle_t FoldbackMotorSpeed;
    Foldback_Handle_t FoldbackMotorTemperature;
    Foldback_Handle_t FoldbackHeatsinkTemperature;
    Foldback_Handle_t FoldbackMotorPower;

    int16_t hCurrentTorqueRef;
    int16_t hCurrentSpeedRef;

    int16_t hFinalTorque;
    int16_t hFinalSpeed;

    STC_Modality_t Mode;   /*!< Modality of STC. It can be one of these two
                               settings: STC_TORQUE_MODE to enable the
                               Torque mode or STC_SPEED_MODE to enable the
                               Speed mode.*/
    PID_Handle_t * pPISpeed;   /*!< The regulator used to perform the speed
                                     control loop.*/
    SpeednPosFdbk_Handle_t * pSPD;/*!< The speed sensor used to perform the speed
                                     regulation.*/
    NTC_Handle_t * pHeatsinkTempSensor; /* Temperature sensor used to monitor heatsink temperature */
    NTC_Handle_t * pMotorTempSensor; /* Temperature sensor used to monitor motor temperature */

    uint16_t hSTCFrequencyHz;             /*!< Frequency on which the user updates
                                             the torque reference calling
                                             SpdTorqCtrl_CalcTorqueReference method
                                             expressed in Hz */
    uint16_t hMaxAppPositiveMecSpeedUnit; /*!< Application maximum positive value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    uint16_t hMinAppPositiveMecSpeedUnit; /*!< Application minimum positive value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    int16_t hMaxAppNegativeMecSpeedUnit;  /*!< Application maximum negative value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    int16_t hMinAppNegativeMecSpeedUnit;  /*!< Application minimum negative value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    uint16_t hMaxPositiveTorque;          /*!< Maximum positive value of motor
                                             torque. This value represents
                                             actually the maximum Iq current
                                             expressed in digit.*/
    int16_t hMinNegativeTorque;           /*!< Minimum negative value of motor
                                             torque. This value represents
                                             actually the maximum Iq current
                                             expressed in digit.*/
    uint16_t hMaxPositivePower;             /*!< Maximum positive value of motor
                                             power in W.*/
    int16_t hMinNegativePower;              /*!< Minimum negative value of motor
                                             power in W.*/

    STC_Modality_t ModeDefault;          /*!< Default STC modality.*/
    int16_t hMecSpeedRefUnitDefault;      /*!< Default mechanical rotor speed
                                             reference expressed in the unit
                                             defined by #SPEED_UNIT.*/
    int16_t hTorqueRefDefault;            /*!< Default motor torque reference.
                                             This value represents actually the
                                             Iq current reference expressed in
                                             digit.*/
    int16_t hIdrefDefault;                /*!< Default Id current reference expressed
                                             in digit.*/
    uint32_t wTorqueSlopePerSecondUp;     /*!< Slope in torque unit per second when ramping up torque. */
    uint32_t wTorqueSlopePerSecondDown;   /*!< Slope in torque unit per second when ramping down torque. */
    uint32_t wSpeedSlopePerSecondUp;      /*!< Slope in #SPEED_UNIT per second when ramping up speed. */
    uint32_t wSpeedSlopePerSecondDown;    /*!< Slope in #SPEED_UNIT per second when ramping down speed. */

    float fGainTorqueIqref;            /* Gain (G) between Iqref in digital amps and torque reference in cNm. Iqref = Torq * G/D  */
    float fGainTorqueIdref;            /* Gain (G) between Idref in digital amps and torque reference in cNm. Idref = Torq * G/D  */

} SpeednTorqCtrlHandle_t;



/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  oPI the PI object used as controller for the speed regulation.
  *         It can be equal to MC_NULL if the STC is initialized in torque mode
  *         and it will never be configured in speed mode.
  * @param  oSPD the speed sensor used to perform the speed regulation.
  *         It can be equal to MC_NULL if the STC is used only in torque
  *         mode.
  * @param  pTempSensorHS the temperature sensor used to monitor heatsink temperature.
  *          If NULL, foldback feature won't take it into consideration.
  * @param  pTempSensorHS the temperature sensor used to monitor heatsink temperature.
  *          If NULL, foldback feature won't take it into consideration.
  * @retval none.
  */
void SpdTorqCtrl_Init(SpeednTorqCtrlHandle_t * pHandle, PID_Handle_t * pPI, SpeednPosFdbk_Handle_t * SPD_Handle,
                        NTC_Handle_t* pTempSensorHS, NTC_Handle_t* pTempSensorMotor);

/**
  * @brief  It should be called before each motor restart. If STC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none.
  */
void SpdTorqCtrl_Clear(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  */
int16_t SpdTorqCtrl_GetMecSpeedRefUnit(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Get the current motor torque reference. This value represents
  *         actually the Iq current reference expressed in digit.
  *         To convert current expressed in digit to current expressed in Amps
  *         is possible to use the formula:
  *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t current motor torque reference. This value represents
  *         actually the Iq current expressed in digit.
  */
int16_t SpdTorqCtrl_GetTorqueRef(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Set the modality of the speed and torque controller. Two modality
  *         are available Torque mode and Speed mode.
  *         In Torque mode is possible to set directly the motor torque
  *         reference or execute a motor torque ramp. This value represents
  *         actually the Iq current reference expressed in digit.
  *         In Speed mode is possible to set the mechanical rotor speed
  *         reference or execute a speed ramp. The required motor torque is
  *         automatically calculated by the STC.
  *         This command interrupts the execution of any previous ramp command
  *         maintaining the last value of Iq.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  bMode modality of STC. It can be one of these two settings:
  *         STC_TORQUE_MODE to enable the Torque mode or STC_SPEED_MODE to
  *         enable the Speed mode.
  * @retval none
  */
void SpdTorqCtrl_SetControlMode(SpeednTorqCtrlHandle_t * pHandle, STC_Modality_t bMode);

/**
  * @brief  Get the modality of the speed and torque controller.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval STC_Modality_t It returns the modality of STC. It can be one of
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t SpdTorqCtrl_GetControlMode(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Starts the execution of a ramp using new target and duration. This
  *         command interrupts the execution of any previous ramp command.
  *         The generated ramp will be in the modality previously set by
  *         SpdTorqCtrl_SetControlMode method.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hTargetFinal final value of command. This is different accordingly
  *         the STC modality.
  *         If STC is in Torque mode hTargetFinal is the value of motor torque
  *         reference at the end of the ramp expressed in cNm.
  *         If STC is in Speed mode hTargetFinal is the value of mechanical
  *         rotor speed reference at the end of the ramp expressed in tenths of
  *         HZ.
  * @retval bool It return false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or max
  *         application torque or below min application speed depending on
  *         current modality of STC) in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true.
  */
bool SpdTorqCtrl_ExecRamp(SpeednTorqCtrlHandle_t * pHandle, int16_t hTargetFinal);

/**
  * @brief  This command interrupts the execution of any previous ramp command.
  *         If STC has been set in Torque mode the last value of Iq is
  *         maintained.
  *         If STC has been set in Speed mode the last value of mechanical
  *         rotor speed reference is maintained.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none
  */
void SpdTorqCtrl_StopRamp(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  It is used to compute the new value of motor torque reference. It
  *         must be called at fixed time equal to hSTCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t motor torque reference. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  */
int16_t SpdTorqCtrl_CalcTorqueReference(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Get the Default mechanical rotor speed reference expressed in tenths
  *         of HZ.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t It returns the Default mechanical rotor speed. reference
  *         expressed in tenths of HZ.
  */
int16_t SpdTorqCtrl_GetMecSpeedRefUnitDefault(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Returns the Application maximum positive value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  */
uint16_t SpdTorqCtrl_GetMaxAppPositiveMecSpeedUnit(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Returns the Application minimum negative value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  */
int16_t SpdTorqCtrl_GetMinAppNegativeMecSpeedUnit(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool SpdTorqCtrl_IsRampCompleted(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief It sets in real time the speed sensor utilized by the STC.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param SPD_Handle Speed sensor component to be set.
  * @retval none
  */
void SpdTorqCtrl_SetSpeedSensor(SpeednTorqCtrlHandle_t * pHandle, SpeednPosFdbk_Handle_t * oSPD);

/**
  * @brief It returns the speed sensor utilized by the FOC.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval SpeednPosFdbk_Handle_t speed sensor utilized by the FOC.
  */
SpeednPosFdbk_Handle_t * SpdTorqCtrl_GetSpeedSensor(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief It returns the default values of Iqdref.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval default values of Iqdref.
  */
qd_t SpdTorqCtrl_GetDefaultIqdref(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Change the nominal current .
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hNominalCurrent This value represents actually the maximum Iq current
            expressed in digit.
  * @retval none
  */
void SpdTorqCtrl_SetNominalCurrent(SpeednTorqCtrlHandle_t * pHandle, uint16_t hNominalCurrent);

/**
  * @brief  Force the speed reference to the curren speed. It is used
  *         at the START_RUN state to initialize the speed reference.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none
  */
void SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrlHandle_t * pHandle);

/**
  * @brief  Set torque ramp slope values, for ramping up and ramping down.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  wTorqueSlopePerSecondUp: Slope value in torque unit per second when ramping up
  * @param  wTorqueSlopePerSecondDown: Slope value in torque unit per second when ramping down
  * @retval none
  */
void SpdTorqCtrl_SetTorqueRampSlope(SpeednTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown);

/**
  * @brief  Set speed ramp slope values, for ramping up and ramping down.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  wSpeedSlopePerSecondUp: Slope value in #SPEED_UNIT per second when ramping up
  * @param  wSpeedSlopePerSecondDown: Slope value in #SPEED_UNIT per second when ramping down
  * @retval none
  */
void SpdTorqCtrl_SetSpeedRampSlope(SpeednTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown);


/**
  * @brief  Get Iq from provided torque reference.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hTorqueRef: Torque reference in cNm
  * @retval int16_t Iq in digital A
  */
int16_t SpdTorqCtrl_GetIqFromTorqueRef(SpeednTorqCtrlHandle_t * pHandle, int16_t hTorqueRef);

/**
  * @brief  Get Id from provided torque reference.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hTorqueRef: Torque reference in cNm
  * @retval int16_t Id in digital A
  */
int16_t SpdTorqCtrl_GetIdFromTorqueRef(SpeednTorqCtrlHandle_t * pHandle, int16_t hTorqueRef);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNTORQCTRLCLASS_H */
