/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#include "pqd_motor_power_measurement.h"
/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1;

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2=
{
  .wConvFact = PQD_CONVERSION_FACTOR2
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2 = &PQD_MotorPowMeasM2;

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX,
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 1
  */
FW_Handle_t FW_M1 =
{
  .hMaxModule             = MAX_MODULE,
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF,
  .hDemagCurrent          = ID_DEMAG,
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  PI Flux Weakening control parameters Motor 1
  */
PID_Handle_t PIDFluxWeakeningHandle_M1 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN,
  .hDefKiGain          = (int16_t)FW_KI_GAIN,
  .wUpperIntegralLimit = 0,
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT) * (int32_t)FW_KIDIV,
  .hUpperOutputLimit       = 0,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)FW_KPDIV,
  .hKiDivisor          = (uint16_t)FW_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 2
  */
FW_Handle_t FW_M2 =
{
  .hMaxModule             = MAX_MODULE2,
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF2,
  .hDemagCurrent          = ID_DEMAG2,
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT2*(int32_t)NOMINAL_CURRENT2),
  .hVqdLowPassFilterBW    = M2_VQD_SW_FILTER_BW_FACTOR,
  .hVqdLowPassFilterBWLOG = M2_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  PI Flux Weakening control parameters Motor 2
  */
PID_Handle_t PIDFluxWeakeningHandle_M2 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN2,
  .hDefKiGain          = (int16_t)FW_KI_GAIN2,
  .wUpperIntegralLimit = 0,
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT2) * (int32_t)FW_KIDIV2,
  .hUpperOutputLimit       = 0,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)FW_KPDIV2,
  .hKiDivisor          = (uint16_t)FW_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FeedForwardCtrl parameters Motor 1
  */
FF_Handle_t FF_M1 =
{
  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
  .wDefConstant_1D        = (int32_t)CONSTANT1_D,
  .wDefConstant_1Q        = (int32_t)CONSTANT1_Q,
  .wDefConstant_2         = (int32_t)CONSTANT2_QD,
  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  FeedForwardCtrl parameters Motor 2
  */
FF_Handle_t FF_M2 =
{
  .hVqdLowPassFilterBW    = M2_VQD_SW_FILTER_BW_FACTOR,
  .wDefConstant_1D        = (int32_t)CONSTANT1_D2,
  .wDefConstant_1Q        = (int32_t)CONSTANT1_Q2,
  .wDefConstant_2         = (int32_t)CONSTANT2_QD2,
  .hVqdLowPassFilterBWLOG = M2_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,
  .ModeDefault =					DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,
};
RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE-1u),
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT,
  .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED_UNIT/2),
  .OTFStartupEnabled       = false,
  .OTFPhaseParams         = {(uint16_t)500,
                                         0,
                             (int16_t)PHASE5_FINAL_CURRENT,
                             (void*)MC_NULL},
  .ParamsData             = {{(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),(int16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
                             {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),(int16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
                             {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),(int16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
                             {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),(int16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
                             {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),(int16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
                            },
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS, STM32G4xx
  */
PWMC_ICS_Handle_t PWM_Handle_M1 = {
  {
    .pFctGetPhaseCurrents              = &ICS_GetPhaseCurrents,
    .pFctSwitchOffPwm                  = &ICS_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &ICS_SwitchOnPWM,
    .pFctCurrReadingCalib              = &ICS_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &ICS_TurnOnLowSides,
    .pFctSetADCSampPointSectX          = &ICS_WriteTIMRegisters,
    .pFctIsOverCurrentOccurred         = &ICS_IsOverCurrentOccurred,
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = MC_NULL,
    .pFctRLDetectionModeDisable        = MC_NULL,
    .pFctRLDetectionModeSetDuty        = MC_NULL,
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false,
    .OffCalibrWaitTimeCounter = 0,
    .Motor = M1,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .DTTest = 0,
    .DTCompCnt = DTCOMPCNT,
    .PWMperiod          = PWM_PERIOD_CYCLES,
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
    .Ton                 = TON,
    .Toff                = TOFF

  },
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
  .PolarizationCounter = 0,
  .OverCurrentFlag = false,
  .OverVoltageFlag = false,
  .BrakeActionLock = false,

  .pParams_str = &ICS_ParamsM1
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 2 - ICS, STM32G4xx
  */
PWMC_ICS_Handle_t PWM_Handle_M2 = {
  {
    .pFctGetPhaseCurrents              = &ICS_GetPhaseCurrents,
    .pFctSwitchOffPwm                  = &ICS_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &ICS_SwitchOnPWM,
    .pFctCurrReadingCalib              = &ICS_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &ICS_TurnOnLowSides,
    .pFctSetADCSampPointSectX          = &ICS_WriteTIMRegisters,
    .pFctIsOverCurrentOccurred         = &ICS_IsOverCurrentOccurred,
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = MC_NULL,
    .pFctRLDetectionModeDisable        = MC_NULL,
    .pFctRLDetectionModeSetDuty        = MC_NULL,
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES2*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false,
    .OffCalibrWaitTimeCounter = 0,
    .Motor = M2,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .DTTest = 0,
    .DTCompCnt = DTCOMPCNT2,
    .PWMperiod          = PWM_PERIOD_CYCLES2,
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000),
    .Ton                 = TON2,
    .Toff                = TOFF2

  },
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES2/2u,
  .PolarizationCounter = 0,
  .OverCurrentFlag = false,
  .OverVoltageFlag = false,
  .BrakeActionLock = false,
  .pParams_str = &ICS_ParamsM2

};

/**
  * @brief  PI / PID Speed loop parameters Motor 2
  */
PID_Handle_t PIDSpeedHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)IQMAX2 * (int32_t)SP_KIDIV2,
  .wLowerIntegralLimit = -(int32_t)IQMAX2 * (int32_t)SP_KIDIV2,
  .hUpperOutputLimit       = (int16_t)IQMAX2,
  .hLowerOutputLimit       = -(int16_t)IQMAX2,
  .hKpDivisor          = (uint16_t)SP_KPDIV2,
  .hKiDivisor          = (uint16_t)SP_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 2
  */
PID_Handle_t PIDIqHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV2,
  .hKiDivisor          = (uint16_t)TF_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 2
  */
PID_Handle_t PIDIdHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV2,
  .hKiDivisor          = (uint16_t)TF_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM2,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor                     =  DPP_CONV_FACTOR2,
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE2,
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE2 * TRANSITION_DURATION2/ 1000.0),
};

/**
  * @brief  SpeednTorque Controller parameters Motor 2
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE2,
  .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT2),
  .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
  .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT2),
  .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT2),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT2,
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT2,
  .ModeDefault =					DEFAULT_CONTROL_MODE2,
  .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT2),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT2,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT2,
};
RevUpCtrl_Handle_t RevUpControlM2 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE2,
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG2)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE2-1u),
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT2,
  .hMinStartUpFlySpeed     = (int16_t)((OBS_MINIMUM_SPEED_UNIT2)/2),
  .OTFStartupEnabled       = true,
  .OTFPhaseParams         = {(uint16_t)500,
                                         0,
                             (int16_t)PHASE5_FINAL_CURRENT2,
                             (void*)MC_NULL},
  .ParamsData             = {{(uint16_t)PHASE1_DURATION2,(int16_t)(PHASE1_FINAL_SPEED_UNIT2),(int16_t)PHASE1_FINAL_CURRENT2,&RevUpControlM2.ParamsData[1]},
                             {(uint16_t)PHASE2_DURATION2,(int16_t)(PHASE2_FINAL_SPEED_UNIT2),(int16_t)PHASE2_FINAL_CURRENT2,&RevUpControlM2.ParamsData[2]},
                             {(uint16_t)PHASE3_DURATION2,(int16_t)(PHASE3_FINAL_SPEED_UNIT2),(int16_t)PHASE3_FINAL_CURRENT2,&RevUpControlM2.ParamsData[3]},
                             {(uint16_t)PHASE4_DURATION2,(int16_t)(PHASE4_FINAL_SPEED_UNIT2),(int16_t)PHASE4_FINAL_CURRENT2,&RevUpControlM2.ParamsData[4]},
                             {(uint16_t)PHASE5_DURATION2,(int16_t)(PHASE5_FINAL_SPEED_UNIT2),(int16_t)PHASE5_FINAL_CURRENT2,(void*)MC_NULL},
                            },

};
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{

  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE,
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - State Observer + CORDIC
  */
STO_CR_Handle_t STO_CR_M2 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM2,
    .SpeedUnit                         =  SPEED_UNIT,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor                     =  DPP_CONV_FACTOR2,
    },
  .hC1                                 =	CORD_C12,
  .hC2                                 =	CORD_C22,
  .hC3                                 =	CORD_C32,
  .hC4                                 =	CORD_C42,
  .hC5                                 =	CORD_C52,
  .hF1                                 =	CORD_F12,
  .hF2                                 =	CORD_F22,
  .SpeedBufferSizeUnit                =	CORD_FIFO_DEPTH_UNIT2,
  .SpeedBufferSizedpp                 =	CORD_FIFO_DEPTH_DPP2,
  .VariancePercentage                 =	CORD_PERCENTAGE_FACTOR2,
  .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT2,
  .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT2,
  .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT2,
  .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS2,
  .Reliability_hysteresys             =	CORD_MEAS_ERRORS_BEFORE_FAULTS2,
  .MaxInstantElAcceleration           =	CORD_MAX_ACCEL_DPPP2,
  .BemfConsistencyCheck               =	CORD_BEMF_CONSISTENCY_TOL2,
  .BemfConsistencyGain                =	CORD_BEMF_CONSISTENCY_GAIN2,
  .MaxAppPositiveMecSpeedUnit         =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT2*1.15),
  .F1LOG                              =	CORD_F1_LOG2 ,
  .F2LOG                              =	CORD_F2_LOG2 ,
  .SpeedBufferSizedppLOG              =	CORD_FIFO_DEPTH_DPP_LOG2
};

STO_Handle_t STO_M2 =
{
  ._Super                        = (SpeednPosFdbk_Handle_t*)&STO_CR_M2,
  .pFctForceConvergency1         = &STO_CR_ForceConvergency1,
  .pFctForceConvergency2         = &STO_CR_ForceConvergency2,
  .pFctStoOtfResetPLL            = MC_NULL,
  .pFctSTO_SpeedReliabilityCheck = &STO_CR_IsSpeedReliable

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + CORDIC
  */
STO_CR_Handle_t STO_CR_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .SpeedUnit                         =  SPEED_UNIT,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
  .hC1                                 =	CORD_C1,
  .hC2                                 =	CORD_C2,
  .hC3                                 =	CORD_C3,
  .hC4                                 =	CORD_C4,
  .hC5                                 =	CORD_C5,
  .hF1                                 =	CORD_F1,
  .hF2                                 =	CORD_F2,
  .SpeedBufferSizeUnit                =	CORD_FIFO_DEPTH_UNIT,
  .SpeedBufferSizedpp                 =	CORD_FIFO_DEPTH_DPP,
  .VariancePercentage                 =	CORD_PERCENTAGE_FACTOR,
  .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,
  .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,
  .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT,
  .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,
  .Reliability_hysteresys             =	CORD_MEAS_ERRORS_BEFORE_FAULTS,
  .MaxInstantElAcceleration           =	CORD_MAX_ACCEL_DPPP,
  .BemfConsistencyCheck               =	CORD_BEMF_CONSISTENCY_TOL,
  .BemfConsistencyGain                =	CORD_BEMF_CONSISTENCY_GAIN,
  .MaxAppPositiveMecSpeedUnit         =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT*1.15),
  .F1LOG                              =	CORD_F1_LOG,
  .F2LOG                              =	CORD_F2_LOG,
  .SpeedBufferSizedppLOG              =	CORD_FIFO_DEPTH_DPP_LOG
};

STO_Handle_t STO_M1 =
{
  ._Super                        = (SpeednPosFdbk_Handle_t*)&STO_CR_M1,
  .pFctForceConvergency1         = &STO_CR_ForceConvergency1,
  .pFctForceConvergency2         = &STO_CR_ForceConvergency2,
  .pFctStoOtfResetPLL            = MC_NULL,
  .pFctSTO_SpeedReliabilityCheck = &STO_CR_IsSpeedReliable

};

/**
  * temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = ADC2,
    .channel = MC_ADC_CHANNEL_12,
    .samplingTime = M1_TEMP_SAMPLING_TIME,
  },
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

/**
  * temperature sensor parameters Motor 2
  */
NTC_Handle_t TempSensorParamsM2 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = ADC1,
    .channel = MC_ADC_CHANNEL_6,
    .samplingTime = M2_TEMP_SAMPLING_TIME,
  },
  .hLowPassFilterBW        = M2_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2 - OV_TEMPERATURE_HYSTERESIS_d2),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT2),
  .wV0                     = (uint16_t)(V0_V2 *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C2,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },

  .VbusRegConv =
  {
    .regADC = ADC1,
    .channel = MC_ADC_CHANNEL_14,
    .samplingTime = M1_VBUS_SAMPLING_TIME,
  },
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,
  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

/**
  * Virtual bus voltage sensor parameters Motor 2
  */
VirtualBusVoltageSensor_Handle_t VirtualBusVoltageSensorParamsM2 =
{
  ._Super =
  {
    .SensorType       = VIRTUAL_SENSOR,
    .ConversionFactor = 500,
  },

  .ExpectedVbus_d = 1+(NOMINAL_BUS_VOLTAGE_V2 * 65536) / 500,
};

UI_Handle_t UI_Params =
{
  .bDriveNum = 0,
  .pFct_DACInit = &DAC_Init,
  .pFct_DACExec = &DAC_Exec,
  .pFctDACSetChannelConfig    = &DAC_SetChannelConfig,
  .pFctDACGetChannelConfig    = &DAC_GetChannelConfig,
  .pFctDACSetUserChannelValue = &DAC_SetUserChannelValue,
  .pFctDACGetUserChannelValue = &DAC_GetUserChannelValue,

};

DAC_UI_Handle_t DAC_UI_Params =
{
  .hDAC_CH1_ENABLED = ENABLE,
  .hDAC_CH2_ENABLED = DISABLE
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,
  .MaxVd          	  = (uint16_t)(MAX_MODULE * FW_VOLTAGE_REF / 1000),
  .Circle_limit_table = MMITABLE,
  .Start_index        = START_INDEX,
};
/** RAMP for Motor2.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM2 =
{
  .FrequencyHz = TF_REGULATION_RATE2
};

/**
  * @brief  CircleLimitation Component parameters Motor 2 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM2 =
{
  .MaxModule          = MAX_MODULE2,
  .MaxVd          	  = (uint16_t)(MAX_MODULE2 * FW_VOLTAGE_REF2 / 1000),
  .Circle_limit_table = MMITABLE2,
  .Start_index        = START_INDEX2,
};

UFCP_Handle_t pUSART =
{
  ._Super.RxTimeout = 0,
  .USARTx = USART1,

};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

