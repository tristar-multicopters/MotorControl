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
#include "board_hardware.h"
#include "ntc_table.h"
#include "board_hardware.h"


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
  .hDefKpGain           = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain           = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit  = (int32_t)SPDCTRL_UPPER_INTEGRAL_LIMIT,
  .wLowerIntegralLimit  = 0, //-(int32_t)SPD_CTRL_MAX_TORQUE * (int32_t)SP_KIDIV,
  .hUpperOutputLimit    = (int32_t)SPDCTRL_UPPER_INTEGRAL_LIMIT,
  .hLowerOutputLimit    = 0, // -(int16_t)SPD_CTRL_MAX_TORQUE,
  .hKpDivisor           = (uint16_t)SP_KPDIV,
  .hKiDivisor           = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2       = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2       = (uint16_t)SP_KIDIV_LOG,
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
  * @brief  Motor Cotrol component parameters Motor 1
  */
MCConfigHandle_t MCConfig =
{
    .hMaxModule             = MAX_MODULE,
    .hDefaultFwVoltRef       = (int16_t)FW_VOLTAGE_REF,
    .hDemagCurrent          = ID_DEMAG,
    .wNominalCurr           = (int32_t)NOMINAL_PEAK_CURRENT,
    .wNominalSqCurr         = ((int32_t)NOMINAL_PEAK_CURRENT*(int32_t)NOMINAL_PEAK_CURRENT),
    .wUsrMaxCurr            = NOMINAL_PEAK_CURRENT,                  //initially programmed with Nominal value
    .hVqdLowPassFilterBw    = M1_VQD_SW_FILTER_BW_FACTOR,
    .hVqdLowPassFilterBwLog = (uint16_t) M1_VQD_SW_FILTER_BW_FACTOR_LOG,           
};

/**
  * @brief  PI Motor Control control parameters Motor 1
  */
PIDHandle_t PIDMotorControlM1 =
{
    .hDefKpGain          = (int16_t)FW_KP_GAIN,
    .hDefKiGain          = (int16_t)FW_KI_GAIN,
    .wUpperIntegralLimit = 0,
    .wLowerIntegralLimit = (int32_t)(-NOMINAL_PEAK_CURRENT) * (int32_t)FW_KIDIV,
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
    .hVqdLowPassFilterBwLog =(uint16_t)M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpdTorqCtrlHandle_t SpeednTorqCtrlM1 =
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
        .hDefaultOutputLimitHigh = NOMINAL_TORQUE,
        .hDefaultOutputLimitLow = 0,
        .hDecreasingEndValue = 10 * FOLDBACK_SPEED_END_VALUE,
        .hDecreasingRange = 10 * FOLDBACK_SPEED_INTERVAL,
    },
        .FoldbackMotorTemperature =
    {
        .bEnableFoldback = true,
        .FoldbackConfig = TRIM,
        .hDefaultOutputLimitHigh = NOMINAL_TORQUE,
        .hDefaultOutputLimitLow = 0,
        .hDecreasingEndValue = 100 * FOLDBACK_MOTOR_TEMP_END_VALUE,
        .hDecreasingRange = 100 * FOLDBACK_MOTOR_TEMP_INTERVAL,
    },
        .FoldbackHeatsinkTemperature =
    {
        .bEnableFoldback = true,
        .FoldbackConfig = TRIM,
        .hDefaultOutputLimitHigh = NOMINAL_TORQUE,
        .hDefaultOutputLimitLow = 0,
        .hDecreasingEndValue = 100 * FOLDBACK_HS_TEMP_END_VALUE, 
        .hDecreasingRange = 100 * FOLDBACK_HS_TEMP_INTERVAL,
    },
    .FoldbackDynamicMaxPower =
    {
        .bEnableFoldback = ENABLE_MAX_POWER_LIMIT,
        .FoldbackConfig = TRIM,
        .hDefaultOutputLimitHigh = MAX_APPLICATION_POSITIVE_POWER,
        .hDefaultOutputLimitLow = MAX_BMS_POSITIVE_POWER,
        .hDecreasingEndValue = MAX_TIME_BMS_TOLERANT,
        .hDecreasingRange = MAX_POWER_LIMIT_TIMEOUT,
    },
    .FoldbackDynamicMaxTorque = 
    {
        .bEnableFoldback = true,
        .FoldbackConfig = SET_THRESHOLD,
        .hDefaultOutputLimitHigh = STARTING_TORQUE,
        .hDefaultOutputLimitLow =  NOMINAL_TORQUE,
        .hDecreasingRange = 10 * DYNAMICTORQUE_THRESHOLD_SPEED,
        .hDecreasingEndValue = 10 * DYNAMICTORQUE_THRESHOLD_SPEED,
    },
    .OCD2_Handle =
    {
        .wPinNumber = CURRENT_SENSOR_OCD2,    // PC14
        .bIsInvertedLogic = true,             //pin is ACTIVE LOW
    },
    .HWOverCurrentDetection = 
    {
        .OCDPowerDeratingSlope = OCD_POWER_DERATING_SLOPE,
        .OCDTimeInterval = OCD_TIME_INTERVAL_COUNTS,
        .OCDPowerDearatingGain = 1,
        .bIsOverCurrentDetected = false,
        .OverCurrentStatus = HARDWARE_OCD
    },
    .DynamicPowerHandle =
    {
        .hDynamicMaxPower = MAX_APPLICATION_POSITIVE_POWER,
        .hBelowMaxPowerTimeout = MAX_POWER_RECOVER_TIMEOUT,  
        .hOverMaxPowerTimeout = 0,    // since using foldback this parameter is not needed
        .hBelowMaxPowerTimer = 0,
        .hOverMaxPowerTimer = 0    
    },
    .StuckProtection =
    {
        .timeout_general = STUCK_TIMER_MAX_COUNTS,
        .timeout_low_battery = STUCK_TIMER_MAX_COUNTS_LOWBATTERY,
        .min_torque = STUCK_MIN_TORQUE,
        .low_battery_voltage = STUCK_LOW_VOLTAGE_THRESHOLD
    },  
    .hSTCFrequencyHz =              MEDIUM_FREQUENCY_TASK_RATE,
    .hMaxBusCurrent =               (uint16_t)(MAX_APPLICATION_CURRENT),
    .hBatteryLowVoltage =           (uint16_t)(LOW_BATTERY_VOLTAGE_THRESHOLD), 
    .hMaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
    .hMinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .hMaxAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
    .hMinAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
    .hMaxPositiveTorque =           (int16_t)NOMINAL_TORQUE,
    .hMinNegativeTorque =           -(int16_t)NOMINAL_TORQUE,
    .hMaxPositivePower =            (int16_t)MAX_APPLICATION_POSITIVE_POWER,
    .hMinNegativePower =            -(int16_t)MAX_APPLICATION_NEGATIVE_POWER,
    .ModeDefault =					DEFAULT_CONTROL_MODE,
    .wTorqueSlopePerSecondUp =      DEFAULT_TORQUE_SLOPE_UP,
    .wTorqueSlopePerSecondDown =    DEFAULT_TORQUE_SLOPE_DOWN,
    .wSpeedSlopePerSecondUp =       DEFAULT_SPEED_SLOPE_UP,
    .wSpeedSlopePerSecondDown =     DEFAULT_SPEED_SLOPE_DOWN,
    .fGainTorqueIqref =             GAIN_TORQUE_IQREF,
    .fGainTorqueIdref =             GAIN_TORQUE_IDREF,
    .fGearRatio =                   MOTOR_GEAR_RATIO,  
    .bEnableSpdLimitControl = ENABLE_SPEED_LIMIT_CONTROL,
    .hSpdLimit = MAX_APPLICATION_SPEED_UNIT,
    .PISpeedLimit =
    {
      .hDefKpGain           = (int16_t)PID_SPEEDLIMIT_KP_DEFAULT,
      .hDefKiGain           = (int16_t)PID_SPEEDLIMIT_KI_DEFAULT,
      .wUpperIntegralLimit  = STARTING_TORQUE * SP_KIDIV,
      .wLowerIntegralLimit  = 0,
      .hUpperOutputLimit    = STARTING_TORQUE,
      .hLowerOutputLimit    = 0,
      .hKpDivisor           = (uint16_t)SP_KPDIV,
      .hKiDivisor           = (uint16_t)SP_KIDIV,
      .hKpDivisorPOW2       = (uint16_t)SP_KPDIV_LOG,
      .hKiDivisorPOW2       = (uint16_t)SP_KIDIV_LOG,
      .hDefKdGain           = 0x0000U,
      .hKdDivisor           = 0x0000U,
      .hKdDivisorPOW2       = 0x0000U,
    },
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

    .IaFilter =
    {
        .pIIRFAInstance = SOCP_IA_IIRFA_HANDLE_ADDRESS,
    },
    .IbFilter =
    {
        .pIIRFAInstance = SOCP_IB_IIRFA_HANDLE_ADDRESS,
    },
    .fCurrentFilterAlpha = CURRENT_FILTER_ALPHA,
    .fCurrentFilterBeta  = CURRENT_FILTER_BETA,

    .hSoftwareOCPMarginCurrent = OCSP_SAFETY_MARGIN,
    .hSoftwareOCPMaximumCurrent = OCSP_MAX_CURRENT,

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
        .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.5*MAX_APPLICATION_SPEED_UNIT),
        .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
        .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
        .hMaxReliableMecAccelUnitP         =	65535,
        .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .hC1                         =	(int16_t) C1,
    .hC2                         =	C2,
    .hC3                         =	(int16_t) C3,
    .hC4                         =	C4,
    .hC5                         =	(int16_t) C5,
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
        .hKpDivisorPOW2 = (uint16_t)PLL_KPDIV_LOG,
        .hKiDivisorPOW2 = (uint16_t)PLL_KIDIV_LOG,
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
    .hF1Log                              =	(uint16_t)F1_LOG,
    .hF2Log                              =	(uint16_t)F2_LOG,
    .hSpeedBufferSizeDppLog              =	(uint16_t)STO_FIFO_DEPTH_DPP_LOG,
    .hForcedDirection                   =  0x0000U
}; BemfObserverPllHandle_t *pBemfObserverPllM1 = &BemfObserverPllM1;

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */
HallPosSensorHandle_t HallPosSensorM1 =
{
    .Super = 
    {
        .bElToMecRatio                     =	POLE_PAIR_NUM,
        .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.5*MAX_APPLICATION_SPEED_UNIT),
        .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
        .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
        .hMaxReliableMecAccelUnitP         =	65535,
        .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .TIMx                = HALL_POSITION_TIMER_HANDLE_ADDRESS,
    .bSensorPlacement     = HALL_SENSORS_PLACEMENT,
    .hPhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
    .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
    .bSpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
    .wTIMClockFreq       = HALL_TIM_CLK,
    .bPWMFreqScaling      = PWM_FREQ_SCALING,
    .bHallMtpa            = HALL_MTPA,
    .H1PortPin = HALL_POSITION_HU_GPIO_PIN,
    .H2PortPin = HALL_POSITION_HV_GPIO_PIN,
    .H3PortPin = HALL_POSITION_HW_GPIO_PIN,
    .fFilterAlpha = MEC_SPEED_FILTER_BUTTERWORTH_ALPHA,
    .fFilterBeta = MEC_SPEED_FILTER_BUTTERWORTH_BETA,
};

/**
  * temperature sensor parameters Motor 1
  */
NTCTempSensorHandle_t TempSensorMotorM1 =
{
    .bSensorType = REAL_SENSOR,
    .TempRegConv =
    {
        .hChannel = MOTOR_TEMP_ANALOG_CHANNEL,
    },
    .hLowPassFilterBw        = M1_TEMP_SW_FILTER_BW_FACTOR,


    .hOverTempThreshold      = (int16_t)(OV_TEMP_MOTOR_THRESHOLD_C),
    .hOverTempDeactThreshold = (int16_t)(OV_TEMP_MOTOR_THRESHOLD_C - OV_TEMP_MOTOR_HYSTERESIS_C),

    .pNTCLookupTable = &MotorNTCLookupTable,
};

/**
  * temperature sensor parameters Controller 1
  */
NTCTempSensorHandle_t TempSensorControllerM1 =
{
    .bSensorType = REAL_SENSOR,
    .TempRegConv =
    {
        .hChannel = HEATSINK_TEMP_ANALOG_CHANNEL,
    },
    .hLowPassFilterBw        = M1_TEMP_SW_FILTER_BW_FACTOR,
    .hOverTempThreshold      = (int16_t)(OV_TEMP_CONTROLLER_THRESHOLD_C),
    .hOverTempDeactThreshold = (int16_t)(OV_TEMP_CONTROLLER_THRESHOLD_C - OV_TEMP_CONTROLLER_HYSTERESIS_C),

    .pNTCLookupTable = &ControllerNTCLookupTable,
};

/* Bus voltage sensor value filter buffer */
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
        .hChannel = BUS_VOLTAGE_ANALOG_CHANNEL,
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

    .pHallSensor = &HallPosSensorM1,

    .hKpGainDef = ROTOR_POS_OBS_KP,
    .hKpDivisorPOW2 = (uint16_t) ROTOR_POS_OBS_KPDIV_LOG,

    .hKiGainDef = ROTOR_POS_OBS_KI,
    .hKiDivisorPOW2 = (uint16_t) ROTOR_POS_OBS_KIDIV_LOG,

    .hKdGainDef = ROTOR_POS_OBS_KD,
    .hKdDivisorPOW2 = (uint16_t) ROTOR_POS_OBS_KDDIV_LOG,

    .fFilterAlpha = MEC_SPEED_FILTER_BUTTERWORTH_ALPHA,
    .fFilterBeta = MEC_SPEED_FILTER_BUTTERWORTH_BETA,
};
