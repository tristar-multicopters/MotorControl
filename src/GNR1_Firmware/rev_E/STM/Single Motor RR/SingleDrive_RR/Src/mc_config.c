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
#include "ntc_table.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

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
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrlHandle_t SpeednTorqCtrlM1 =
{
  .TorqueRampMngr =
  {
    .wScalingFactor = INT16_MAX,
  },
  .SpeedRampMngr =
  {
    .wScalingFactor = INT16_MAX,
  },
  .FoldbackMotorSpeed =
  {
      .bEnableFoldback = true,
      .FoldbackConfig = TRIM,
      .hDefaultOutputLimitHigh = STARTING_TORQUE,
      .hDefaultOutputLimitLow = 0,
      .hDecreasingEndValue = MAX_APPLICATION_SPEED_UNIT,
      .hDecreasingRange = MAX_APPLICATION_SPEED_UNIT/5,
  },
  .FoldbackMotorTemperature =
  {
      .bEnableFoldback = false,
      .FoldbackConfig = TRIM,
      .hDefaultOutputLimitHigh = STARTING_TORQUE,
      .hDefaultOutputLimitLow = 0,
      .hDecreasingEndValue = 0,
      .hDecreasingRange = 0,
  },
  .FoldbackHeatsinkTemperature =
  {
      .bEnableFoldback = true,
      .FoldbackConfig = TRIM,
      .hDefaultOutputLimitHigh = STARTING_TORQUE,
      .hDefaultOutputLimitLow = 0,
      .hDecreasingEndValue = OV_TEMPERATURE_THRESHOLD_C * 100,          // This is due to limitation of Foldback module
      .hDecreasingRange = (OV_TEMPERATURE_THRESHOLD_C/2) * 100 ,        // This is due to limitation of Foldback module
  },
  .FoldbackMotorPower =
  {
      .bEnableFoldback = false,
      .FoldbackConfig = TRIM,
      .hDefaultOutputLimitHigh = STARTING_TORQUE,
      .hDefaultOutputLimitLow = 0,
      .hDecreasingEndValue = MOTOR_MAX_POWER,
      .hDecreasingRange = MOTOR_MAX_POWER/2,
  },
  .FoldbackDymamicMaxTorque = 
  {
      .bEnableFoldback = true,
      .FoldbackConfig = SET_THRESHOLD,
      .hDefaultOutputLimitHigh = STARTING_TORQUE,
      .hDefaultOutputLimitLow = NOMINAL_TORQUE,
      .hDecreasingEndValue = DYMANIC_TORQUE_END_SPEED,
      .hDecreasingRange = DYMANIC_TORQUE_END_SPEED/2,
  },
  
  .hSTCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE,
  .hMaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .hMinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .hMaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .hMinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .hMaxPositiveTorque =				(int16_t)STARTING_TORQUE,
  .hMinNegativeTorque =				-(int16_t)STARTING_TORQUE,
  .hMaxPositivePower =				(int16_t)MAX_APPLICATION_POSITIVE_POWER,
  .hMinNegativePower =				-(int16_t)MAX_APPLICATION_NEGATIVE_POWER,
  .ModeDefault =					DEFAULT_CONTROL_MODE,
  .hMecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .hTorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .hIdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,
  .wTorqueSlopePerSecondUp =    DEFAULT_TORQUE_SLOPE_UP,
  .wTorqueSlopePerSecondDown =  DEFAULT_TORQUE_SLOPE_DOWN,
  .wSpeedSlopePerSecondUp =     DEFAULT_SPEED_SLOPE_UP,
  .wSpeedSlopePerSecondDown =   DEFAULT_SPEED_SLOPE_DOWN,

  .fGainTorqueIqref =           GAIN_TORQUE_IQREF,
  .fGainTorqueIdref =           GAIN_TORQUE_IDREF,
	.SpeedFilter =
    {
        0
    },
  .hSpeedFilterLength =  SPEED_FILTER_LENGTH,	
	
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
    .Toff                = TOFF,

    .IaFilter =
    {
        0
    },
    .IbFilter =
    {
        0
    },
    .fCurrentFilterAlpha = CURRENT_FILTER_ALPHA,
    .fCurrentFilterBeta  = CURRENT_FILTER_BETA,

    .hSoftwareOCPMarginCurrent = OCSP_SAFETY_MARGIN,
    .hSoftwareOCPMaximumCurrent = OCSP_MAX_CURRENT,

  },
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
  .PolarizationCounter = 0,
  .HwOverCurrentFlag = false,
  .SwOverCurrentFlag = false,
  .OverVoltageFlag = false,
  .BrakeActionLock = false,

  .pParams_str = &ICS_ParamsM1
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + PLL
  */
STO_PLL_Handle_t STO_PLL_M1 =
{
  ._Super = {
	.bElToMecRatio                     =	POLE_PAIR_NUM,
    .SpeedUnit                         = SPEED_UNIT,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },
 .hC1                         =	C1,
 .hC2                         =	C2,
 .hC3                         =	C3,
 .hC4                         =	C4,
 .hC5                         =	C5,
 .hF1                         =	F1,
 .hF2                         =	F2,
 .PIRegulator = {
     .hDefKpGain = PLL_KP_GAIN,
     .hDefKiGain = PLL_KI_GAIN,
	 .hDefKdGain = 0x0000U,
     .hKpDivisor = PLL_KPDIV,
     .hKiDivisor = PLL_KIDIV,
	 .hKdDivisor = 0x0000U,
     .wUpperIntegralLimit = INT32_MAX,
     .wLowerIntegralLimit = -INT32_MAX,
     .hUpperOutputLimit = INT16_MAX,
     .hLowerOutputLimit = -INT16_MAX,
     .hKpDivisorPOW2 = PLL_KPDIV_LOG,
     .hKiDivisorPOW2 = PLL_KIDIV_LOG,
     .hKdDivisorPOW2       = 0x0000U,
   },
 .SpeedBufferSizeUnit                =	STO_FIFO_DEPTH_UNIT,
 .SpeedBufferSizeDpp                 =	STO_FIFO_DEPTH_DPP,
 .VariancePercentage                 =	PERCENTAGE_FACTOR,
 .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,
 .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,
 .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT,
 .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,
 .Reliability_hysteresys             =	OBS_MEAS_ERRORS_BEFORE_FAULTS,
 .BemfConsistencyCheck               =	BEMF_CONSISTENCY_TOL,
 .BemfConsistencyGain                =	BEMF_CONSISTENCY_GAIN,
 .MaxAppPositiveMecSpeedUnit         =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT*1.15),
 .F1LOG                              =	F1_LOG,
 .F2LOG                              =	F2_LOG,
 .SpeedBufferSizeDppLOG              =	STO_FIFO_DEPTH_DPP_LOG,
 .hForcedDirection                   =  0x0000U
};
STO_PLL_Handle_t *pSTO_PLL_M1 = &STO_PLL_M1;

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */

HALL_Handle_t HALL_M1 =
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
  .SensorPlacement     = HALL_SENSORS_PLACEMENT,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
 .TIMClockFreq       = HALL_TIM_CLK,
 .TIMx                = TIM2,

 .ICx_Filter          = M1_HALL_IC_FILTER,

 .PWMFreqScaling      = PWM_FREQ_SCALING,
 .HallMtpa            = HALL_MTPA,

 .H1Port             =  M1_HALL_H1_GPIO_Port,
 .H1Pin              =  M1_HALL_H1_Pin,
 .H2Port             =  M1_HALL_H2_GPIO_Port,
 .H2Pin              =  M1_HALL_H2_Pin,
 .H3Port             =  M1_HALL_H3_GPIO_Port,
 .H3Pin              =  M1_HALL_H3_Pin,
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

  .pNTCLookupTable = &NTCLookupTable,
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
  .hDAC_CH2_ENABLED = ENABLE
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

UFCP_Handle_t pUSART =
{
  ._Super.RxTimeout = 0,
  .USARTx = USART1,

};

/* USER CODE BEGIN Additional configuration */

RotorPositionObserverHandle_t RotorPosObsM1 =
{
  .Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.5*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },

  .pHallSensor = &HALL_M1,

	.hKpGainDef = ROTOR_POS_OBS_KP,
	.hKpDivisorPOW2 = ROTOR_POS_OBS_KPDIV_LOG,

	.hKiGainDef = ROTOR_POS_OBS_KI,
	.hKiDivisorPOW2 = ROTOR_POS_OBS_KIDIV_LOG,

	.hKdGainDef = ROTOR_POS_OBS_KD,
	.hKdDivisorPOW2 = ROTOR_POS_OBS_KDDIV_LOG,
};


/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
