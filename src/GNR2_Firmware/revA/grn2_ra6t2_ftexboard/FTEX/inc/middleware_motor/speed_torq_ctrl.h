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
#include "dynamic_power.h"
#include "stuck_protection.h"


/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Speed & Torque Control parameters definition
  */
typedef struct
{
    RampMngr_Handle_t TorqueRampMngr;                   /* Ramp management structure for torque reference */
    RampMngr_Handle_t SpeedRampMngr;                    /* Ramp management structure for speed reference */

    Foldback_Handle_t FoldbackMotorSpeed;               /* Foldback structure used to limit maximum motor speed */
    Foldback_Handle_t FoldbackMotorTemperature;         /* Foldback structure used to limit maximum motor temperature */
    Foldback_Handle_t FoldbackHeatsinkTemperature;      /* Foldback structure used to limit maximum heatsink temperature */
    Foldback_Handle_t FoldbackDynamicMaxPower;         /* Foldback structure used to limit maximum Power after a period of time */

    
    Foldback_Handle_t FoldbackDynamicMaxTorque;      /* Foldback structure used to limit maximum Torque to other foldbacks */   

    DynamicPowerHandle_t   DynamicPowerHandle;
    
    int16_t hCurrentTorqueRef;
    int16_t hCurrentSpeedRef;

    int16_t hFinalTorqueRef;
    int16_t hFinalSpeedRef;

    STCModality_t                   Mode;        /*!< Modality of STC. It can be one of these two
                                                settings: STC_TORQUE_MODE to enable the
                                                Torque mode or STC_SPEED_MODE to enable the Speed mode.*/
    PIDHandle_t                     * pPISpeed;   /*!< The regulator used to perform the speed
                                     control loop.*/
    SpdPosFdbkHandle_t              * pSPD;/*!< The speed sensor used to perform the speed
                                     regulation.*/
    NTCTempSensorHandle_t           * pHeatsinkTempSensor; /* Temperature sensor used to monitor heatsink temperature */
    NTCTempSensorHandle_t           * pMotorTempSensor; /* Temperature sensor used to monitor motor temperature */
    StuckProtection_t               StuckProtection; /* parameters of Stcuk Protection */
    uint16_t hSTCFrequencyHz;               /*!< Frequency on which the user updates
                                             the torque reference calling
                                             SpdTorqCtrl_CalcTorqueReference method
                                             expressed in Hz */
    uint16_t hBusVoltage;                   /* the Bus Voltage coming from Voltage Sensor
                                             in voltage unit                                            */    
    uint16_t hMaxBusCurrent;                /*!< Application maximum Peak Current
                                             of the rotor mechanical speed. Expressed in Amps 
                                             */
    uint16_t hMaxContinuousCurrent;         /*!< Application maximumContinouse Current
                                             of the rotor mechanical speed. Expressed in Amps 
                                             */
    bool hEnableLVtorqueLimit;              /* Enable or disable the low voltage torque limit*/ 
    uint16_t hBatteryLowVoltage;            /* Application maximum voltage that the MC layer can 
                                              operate with maximum torque
                                              */
    uint16_t hMaxAppPositiveMecSpeedUnit;   /*!< Application maximum positive value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    uint16_t hMinAppPositiveMecSpeedUnit;   /*!< Application minimum positive value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    int16_t hMaxAppNegativeMecSpeedUnit;    /*!< Application maximum negative value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    int16_t hMinAppNegativeMecSpeedUnit;    /*!< Application minimum negative value
                                             of the rotor mechanical speed. Expressed in
                                             the unit defined by #SPEED_UNIT.*/
    uint16_t hMaxPositiveTorque;            /*!< Maximum positive value of motor
                                             torque in cNm.*/
    int16_t  hMinNegativeTorque;            /*!< Minimum negative value of motor
                                             torque in cNm.*/                         
    uint16_t hMaxPositivePower;             /*!< Maximum positive value of motor
                                             power in W.*/
    uint16_t hMaxContinuousPower;             /*!< Maximum positive value of motor
                                             power in W.*/
    int16_t hMinNegativePower;              /*!< Minimum negative value of motor
                                             power in W.*/
    STCModality_t ModeDefault;              /*!< Default STC modality.*/
    uint32_t wTorqueSlopePerSecondUp;       /*!< Slope in cNm per second when ramping up torque. */
    uint32_t wTorqueSlopePerSecondDown;     /*!< Slope in cNm per second when ramping down torque. */
    uint32_t wSpeedSlopePerSecondUp;        /*!< Slope in #SPEED_UNIT per second when ramping up speed. */
    uint32_t wSpeedSlopePerSecondDown;      /*!< Slope in #SPEED_UNIT per second when ramping down speed. */

    float fGainTorqueIqref;            /* Gain (G) between Iqref in digital amps and torque reference in cNm. Iqref = Torq * G/D  */
    float fGainTorqueIdref;            /* Gain (G) between Idref in digital amps and torque reference in cNm. Idref = Torq * G/D  */
    
    float fGearRatio;                  /* fGearRatio Contians the gear ratio of the motor as defined in drive_parameters_xxx*/    
    
    bool bEnableSpdLimitControl;
    int16_t hSpdLimit;
    int16_t hTorqueReferenceSpdLim;
    PIDHandle_t PISpeedLimit;               /*!< The regulator used to perform the speed limit control loop.*/
    
    
} SpdTorqCtrlHandle_t;







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
void SpdTorqCtrl_Init(SpdTorqCtrlHandle_t * pHandle, PIDHandle_t * pPI, SpdPosFdbkHandle_t * SPD_Handle,
                        NTCTempSensorHandle_t* pTempSensorHS, NTCTempSensorHandle_t* pTempSensorMotor);

/**
  * @brief  It should be called before each motor restart. If STC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none.
  */
void SpdTorqCtrl_Clear(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  */
int16_t SpdTorqCtrl_GetMecSpeedRefUnit(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Get the current motor torque reference in cNm (Nm/100).
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t current motor torque reference in cNm.
  */
int16_t SpdTorqCtrl_GetTorqueRef(SpdTorqCtrlHandle_t * pHandle);

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
void SpdTorqCtrl_SetControlMode(SpdTorqCtrlHandle_t * pHandle, STCModality_t bMode);

/**
  * @brief  Get the modality of the speed and torque controller.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval STCModality_t It returns the modality of STC. It can be one of
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STCModality_t SpdTorqCtrl_GetControlMode(SpdTorqCtrlHandle_t * pHandle);

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
bool SpdTorqCtrl_ExecRamp(SpdTorqCtrlHandle_t * pHandle, int16_t hTargetFinal);

/**
  * @brief  This command interrupts the execution of any previous ramp command.
  *         If STC has been set in Torque mode the last value of torque is
  *         maintained.
  *         If STC has been set in Speed mode the last value of mechanical
  *         rotor speed reference is maintained.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none
  */
void SpdTorqCtrl_StopRamp(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  It is used to compute the new value of motor torque reference. It
  *         must be called at fixed time equal to hSTCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t motor torque reference in cNm (Nm/100).
  */
int16_t SpdTorqCtrl_CalcTorqueReference(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Returns the Application maximum positive value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  */
uint16_t SpdTorqCtrl_GetMaxAppPositiveMecSpeedUnit(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Returns the Application minimum negative value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  */
int16_t SpdTorqCtrl_GetMinAppNegativeMecSpeedUnit(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool SpdTorqCtrl_IsRampCompleted(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief It sets in real time the speed sensor utilized by the STC.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param SPD_Handle Speed sensor component to be set.
  * @retval none
  */
void SpdTorqCtrl_SetSpeedSensor(SpdTorqCtrlHandle_t * pHandle, SpdPosFdbkHandle_t * oSPD);

/**
  * @brief It returns the speed sensor utilized by the FOC.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval SpdPosFdbkHandle_t speed sensor utilized by the FOC.
  */
SpdPosFdbkHandle_t * SpdTorqCtrl_GetSpeedSensor(SpdTorqCtrlHandle_t * pHandle);


/**
  * @brief  Force the speed reference to the curren speed. It is used
  *         at the START_RUN state to initialize the speed reference.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none
  */
void SpdTorqCtrl_ForceSpeedReferenceToCurrentSpeed(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Set torque ramp slope values, for ramping up and ramping down.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  wTorqueSlopePerSecondUp: Slope value in torque unit per second when ramping up
  * @param  wTorqueSlopePerSecondDown: Slope value in torque unit per second when ramping down
  * @retval none
  */
void SpdTorqCtrl_SetTorqueRampSlope(SpdTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown);

/**
  * @brief  Set speed ramp slope values, for ramping up and ramping down.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  wSpeedSlopePerSecondUp: Slope value in #SPEED_UNIT per second when ramping up
  * @param  wSpeedSlopePerSecondDown: Slope value in #SPEED_UNIT per second when ramping down
  * @retval none
  */
void SpdTorqCtrl_SetSpeedRampSlope(SpdTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown);

/**
  * @brief  Set speed ramp slope values, for ramping up and ramping down.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  wSpeedSlopePerSecondUp: Slope value in #SPEED_UNIT per second when ramping up
  * @param  wSpeedSlopePerSecondDown: Slope value in #SPEED_UNIT per second when ramping down
  * @retval none
  */
void SpdTorqCtrl_SetSpeedRampSlope(SpdTorqCtrlHandle_t * pHandle, uint32_t wSlopePerSecondUp, uint32_t wSlopePerSecondDown);

/**
  * @brief  Get Iq from provided torque reference.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hTorqueRef: Torque reference in cNm
  * @retval int16_t Iq in digital A
  */
int16_t SpdTorqCtrl_GetIqFromTorqueRef(SpdTorqCtrlHandle_t * pHandle, int16_t hTorqueRef);

/**
  * @brief  Get Id from provided torque reference.
  *         To check motor speed, if it is zero leave counter to rise value
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hTorqueRef: Torque reference in cNm
  * @retval int16_t Id in digital A
  */
int16_t SpdTorqCtrl_GetIdFromTorqueRef(SpdTorqCtrlHandle_t * pHandle, int16_t hTorqueRef);

/**
  * @brief  Get Id, Iq from provided 
  *         to limit them to hMaxCurrent
  * @param  pHandle: handler of the current instance of the FOCVars component
  * @retval null
  */
void SpdTorqCtrl_ApplyCurrentLimitation_Iq(qd_t * pHandle, int16_t NominalCurr, int16_t UsrMaxCurr);

/**
  * @brief  Sets Motor Max Power based if it is defined to be based on Battery SoC
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval 
  */
void MC_AdaptiveMaxPower(SpdTorqCtrlHandle_t * pHandle);

/**
  * @brief  Sets motor speed limit for speed limit controller (only for torque mode).
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hSpdLim: Speed limit in unit specified by SPEED_UNIT (rpm, Hz, ...)
  * @retval 
  */
void SpdTorqCtrl_SetSpeedLimit(SpdTorqCtrlHandle_t * pHandle, int16_t hSpdLimUnit);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNTORQCTRLCLASS_H */
