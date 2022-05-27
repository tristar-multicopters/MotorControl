/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  FTEX inc
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
*/

#include "gnr_main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "pqd_motor_power_measurement.h"
#include "mc_config.h"

#define OFFCALIBRWAIT_MS     0
#define MAX_DUTY     				 30000 /* INT16_MAX is 100% duty cycle */

//PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
//{
//  .wConvFact = PQD_CONVERSION_FACTOR
//};
//PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1;

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
PID_Handle_t PIDIdHandle_M1 =
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

///**
//  * @brief  FluxWeakeningCtrl component parameters Motor 1
//  */
//FW_Handle_t FW_M1 =
//{
//  .hMaxModule             = MAX_MODULE,
//  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF,
//  .hDemagCurrent          = ID_DEMAG,
//  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
//  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
//  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
//};

///**
//  * @brief  PI Flux Weakening control parameters Motor 1
//  */
//PID_Handle_t PIDFluxWeakeningHandle_M1 =
//{
//  .hDefKpGain          = (int16_t)FW_KP_GAIN,
//  .hDefKiGain          = (int16_t)FW_KI_GAIN,
//  .wUpperIntegralLimit = 0,
//  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT) * (int32_t)FW_KIDIV,
//  .hUpperOutputLimit       = 0,
//  .hLowerOutputLimit       = -INT16_MAX,
//  .hKpDivisor          = (uint16_t)FW_KPDIV,
//  .hKiDivisor          = (uint16_t)FW_KIDIV,
//  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG,
//  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG,
//  .hDefKdGain           = 0x0000U,
//  .hKdDivisor           = 0x0000U,
//  .hKdDivisorPOW2       = 0x0000U,
//};

///**
//  * @brief  FeedForwardCtrl parameters Motor 1
//  */
//FF_Handle_t FF_M1 =
//{
//  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
//  .wDefConstant_1D        = (int32_t)CONSTANT1_D,
//  .wDefConstant_1Q        = (int32_t)CONSTANT1_Q,
//  .wDefConstant_2         = (int32_t)CONSTANT2_QD,
//  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
//};

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
    .Motor = M1,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .PWMperiod          = PWM_PERIOD_CYCLES,

  },
	.hIaRaw = 0,
	.hIbRaw = 0,
	
	.bOverrunFlag = false,
	
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
  .PolarizationCounter = 0,
  .bOverCurrentFlag = false,

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

///**
//  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
//  */

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
  .TIMx                = &g_timer0, 
  .SensorPlacement     = HALL_SENSORS_PLACEMENT,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
  .TIMClockFreq       = HALL_TIM_CLK,
  .PWMFreqScaling      = PWM_FREQ_SCALING,
  .HallMtpa            = HALL_MTPA,
  .H1PortPin = M1_ENC_U,
  .H2PortPin = M1_ENC_V,
  .H3PortPin = M1_ENC_W,
};

/**
  * temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = &g_adc,
    .channel = MC_ADC_CHANNEL_6,
    .group = GROUP_1,
  },
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
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
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },

  .VbusRegConv =
  {
    .regADC = &g_adc,
    .channel = MC_ADC_CHANNEL_7,
    .group = GROUP_1,
  },
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,
  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
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

AO_Handle_t AngleObserverM1 =
{
  .pHallFdbk = &HALL_M1,
	
	.hKpGainDef = AO_KP,
	.hKpDivisor = AO_KPDIV,
	.hKpDivisorPOW2 = AO_KPDIV_LOG,
	
	.hKiGainDef = AO_KI,
	.hKiDivisor = AO_KIDIV,
	.hKiDivisorPOW2 = AO_KIDIV_LOG,
	
	.hKdGainDef = AO_KD,
	.hKdDivisor = AO_KDDIV,
	.hKdDivisorPOW2 = AO_KDDIV_LOG,
	
	.hSpeedFactorGain = AO_SPEEDFACTORGAIN,
	.hSpeedFactorDiv = AO_SPEEDFACTORDIV,
};


