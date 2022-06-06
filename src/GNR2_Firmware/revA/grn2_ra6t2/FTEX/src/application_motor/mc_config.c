/**
  * @file    mc_config.c
  * @brief   Motor Control Subsystem components configuration and handler structures.
*/

#include "gnr_main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "pqd_motor_power_measurement.h"
#include "mc_config.h"

#define OFFCALIBRWAIT_MS     0
#define MAX_DUTY     				 30000 /* INT16_MAX is 100% duty cycle */

MotorPowerQDHandle_t PQDMotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
MotorPowerQDHandle_t *pPQD_MotorPowMeasM1 = &PQDMotorPowMeasM1;

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PIDHandle_t PIDSpeedHandleM1 =
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
PIDHandle_t PIDIqHandleM1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)MAX_DUTY * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-MAX_DUTY * TF_KIDIV,
  .hUpperOutputLimit       = MAX_DUTY,
  .hLowerOutputLimit       = -MAX_DUTY,
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
PIDHandle_t PIDIdHandleM1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)MAX_DUTY * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-MAX_DUTY * TF_KIDIV,
  .hUpperOutputLimit       = MAX_DUTY,
  .hLowerOutputLimit       = -MAX_DUTY,
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
FluxWeakeningHandle_t FluxWeakeningM1 =
{
  .hMaxModule             = MAX_MODULE,
  .hDefaultFwVoltRef       = (int16_t)FW_VOLTAGE_REF,
  .hDemagCurrent          = ID_DEMAG,
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
  .hVqdLowPassFilterBw    = M1_VQD_SW_FILTER_BW_FACTOR,
  .hVqdLowPassFilterBwLog = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  PI Flux Weakening control parameters Motor 1
  */
PIDHandle_t PIDFluxWeakeningHandleM1 =
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
FeedforwardHandle_t FeedforwardM1 =
{
  .hVqdLowPassFilterBw    = M1_VQD_SW_FILTER_BW_FACTOR,
  .wDefConstant1D        = (int32_t)CONSTANT1_D,
  .wDefConstant1Q        = (int32_t)CONSTANT1_Q,
  .wDefConstant2         = (int32_t)CONSTANT2_QD,
  .hVqdLowPassFilterBwLog = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrlHandle_t SpeednTorqCtrlM1 =
{
  .hSTCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE,
  .hMaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .hMinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .hMaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .hMinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .hMaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,
  .hMinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,
  .ModeDefault =					DEFAULT_CONTROL_MODE,
  .hMecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .hTorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .hIdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS, STM32G4xx
  */
PWMInsulCurrSensorFdbkHandle_t PWMInsulCurrSensorFdbkHandleM1 = {
  {
    .pFctGetPhaseCurrents              = &PWMInsulCurrSensorFdbk_GetPhaseCurrents,
    .pFctSwitchOffPwm                  = &PWMInsulCurrSensorFdbk_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &PWMInsulCurrSensorFdbk_SwitchOnPWM,
    .pFctCurrReadingCalib              = &PWMInsulCurrSensorFdbk_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &PWMInsulCurrSensorFdbk_TurnOnLowSides,
    .pFctSetADCSampPointSectX          = &PWMInsulCurrSensorFdbk_WriteTIMRegisters,
    .pFctIsOverCurrentOccurred         = &PWMInsulCurrSensorFdbk_IsOverCurrentOccurred,
    .pFctRLDetectionModeEnable         = MC_NULL,
    .pFctRLDetectionModeDisable        = MC_NULL,
    .pFctRLDetectionModeSetDuty        = MC_NULL,
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .hCntPhA = 0,
    .hCntPhB = 0,
    .hCntPhC = 0,
    .hSWerror = 0,
    .hTurnOnLowSidesAction = false,
    .Motor = M1,
    .bRLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .hPWMperiod          = PWM_PERIOD_CYCLES,

  },
	.hIaRaw = 0,
	.hIbRaw = 0,

	.bOverrunFlag = false,

  .wPhaseAOffset = 0,
  .wPhaseBOffset = 0,
  .hHalfPWMPeriod = PWM_PERIOD_CYCLES/2u,
  .bPolarizationCounter = 0,
  .bOverCurrentFlag = false,

  .pParamsStructure = &PWMICSParamsM1
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + PLL
  */
BemfObserverPllHandle_t BemfObserverPllM1 =
{
  .Super = {
	.bElToMecRatio                     =	POLE_PAIR_NUM,
    .bSpeedUnit                         =    SPEED_UNIT,
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
 .bSpeedBufferSizeUnit                =	STO_FIFO_DEPTH_UNIT,
 .bSpeedBufferSizeDpp                 =	STO_FIFO_DEPTH_DPP,
 .hVariancePercentage                 =	PERCENTAGE_FACTOR,
 .bSpeedValidationBandHigh              =	SPEED_BAND_UPPER_LIMIT,
 .bSpeedValidationBandLow              =	SPEED_BAND_LOWER_LIMIT,
 .hMinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT,
 .bStartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,
 .bReliabilityHysteresys             =	OBS_MEAS_ERRORS_BEFORE_FAULTS,
 .bBemfConsistencyCheck               =	BEMF_CONSISTENCY_TOL,
 .bBemfConsistencyGain                =	BEMF_CONSISTENCY_GAIN,
 .hMaxAppPositiveMecSpeedUnit         =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT*1.15),
 .hF1Log                              =	F1_LOG,
 .hF2Log                              =	F2_LOG,
 .hSpeedBufferSizeDppLog              =	STO_FIFO_DEPTH_DPP_LOG,
 .hForcedDirection                   =  0x0000U
}; BemfObserverPllHandle_t *pBemfObserverPllM1 = &BemfObserverPllM1;

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */
HallPosSensorHandle_t HallPosSensorM1 =
{
  .Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },
  .TIMx                = &g_timer0,
  .bSensorPlacement     = HALL_SENSORS_PLACEMENT,
  .hPhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .bSpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
  .wTIMClockFreq       = HALL_TIM_CLK,
  .bPWMFreqScaling      = PWM_FREQ_SCALING,
  .bHallMtpa            = HALL_MTPA,
  .H1PortPin = M1_ENC_U,
  .H2PortPin = M1_ENC_V,
  .H3PortPin = M1_ENC_W,
};

/**
  * temperature sensor parameters Motor 1
  */
NTCTempSensorHandle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = &g_adc,
    .channel = ADC_CHANNEL_0,
    .group = GROUP_1,
  },
  .hLowPassFilterBw        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

///* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
ResDivVbusSensorHandle_t RealBusVoltageSensorParamsM1 =
{
  .Super                =
  {
    .SensorType          = REAL_SENSOR,
    .hConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },

  .VbusRegConv =
  {
    .regADC = &g_adc,
    .channel = ADC_CHANNEL_1,
    .group = GROUP_1,
  },
  .hLowPassFilterBw       =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .hOverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,
  .hUnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitationHandle_t CircleLimitationM1 =
{
  .hMaxModule          = MAX_MODULE,
  .hMaxVd          	  = (uint16_t)(MAX_MODULE * FW_VOLTAGE_REF / 1000),
  .CircleLimitTable = MMITABLE,
  .bStartIndex        = START_INDEX,
};

RotorPositionObserverHandle_t AngleObserverM1 =
{
  .pHallFdbk = &HallPosSensorM1,

	.hKpGainDef = AO_KP,
	.hKpDivisorPOW2 = AO_KPDIV_LOG,

	.hKiGainDef = AO_KI,
	.hKiDivisorPOW2 = AO_KIDIV_LOG,

	.hKdGainDef = AO_KD,
	.hKdDivisorPOW2 = AO_KDDIV_LOG,
};
