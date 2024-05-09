/**
  * @file    mc_config.c
  * @brief   Motor Control Subsystem components configuration and handler structures.
*/

#include "gnr_main.h"
#include "mc_type.h"
#include "pqd_motor_power_measurement.h"
#include "mc_config.h"
#include "board_hardware.h"
#include "ntc_table.h"
#include "board_hardware.h"
#include "motor_parameters.h"

#define OFFCALIBRWAIT_MS     0

/**
  * @brief  Motor Parameters to initialize the motor
  */
MotorParameters_t MotorParameters =
{
    .ConfigParameters =
    {
        .fMotorGearRatio                = MOTOR_GEAR_RATIO,
        .bMotorType                     = MOTOR_TYPE,
        .bPolePairNum                   = POLE_PAIR_NUM,
        .fRS                            = RS_VAL,
        .fLS                            = LS,
        .fMotorMagnetFlux               = MOTOR_MAGNET_FLUX,
        .fMotorVotlageConstant          = MOTOR_VOLTAGE_CONSTANT,
        .fSTTorqueCoef                  = ST_TORQUE_COEF, 
        
        .hPeakCurrentMotorAmps          = PEAK_CURRENT_MOTOR_amps,

    },
    .PowerParameters =
    {   
        .hEstimatedEfficiency           = ESTIMATED_EFFICIENCY,
    },
    .SpeedParameters =
    {
        
        .hMaxAppliationSpeedRPM         = MAX_APPLICATION_SPEED_RPM,
        
        .bEnableSpeedLimitControl       = ENABLE_SPEED_LIMIT_CONTROL,
        .hPIDSpeedKpDefault             = PID_SPEED_KP_DEFAULT,
        .hPIDSpeedKiDefault             = PID_SPEED_KI_DEFAULT,
        .hSpKiDiv                       = SP_KIDIV,
        .hSpKiDivLog                    = (uint16_t) SP_KIDIV_LOG,
        
        .hFoldbackSpeedInterval         = FOLDBACK_SPEED_INTERVAL,
    
    },
    .TorqueParameters = 
    {
        .hPIDTorqueKpDefault            = PID_TORQUE_KP_DEFAULT,
    },
    .FluxParameters = 
    {
        .bFluxWeakeningEnable           = FLUX_WEAKENING_ENABLE,
        
        .hPIDFluxKPDefault              = PID_FLUX_KP_DEFAULT,
        .hPIDFluxKIDefault              = PID_FLUX_KI_DEFAULT,
    },
    .RampManagerParameters =
    {
        .wDefaultTorqueSlopeUp          = DEFAULT_TORQUE_SLOPE_UP,
        .wDefaultTorqueSlopeDown        = DEFAULT_TORQUE_SLOPE_DOWN,
        .wDefaultSpeedSlopeUp           = DEFAULT_SPEED_SLOPE_UP,
        .wDefaultSpeedSlopeDown         = DEFAULT_SPEED_SLOPE_DOWN,
        
        .fMecSpeedFilterButterworthAlpha = MEC_SPEED_FILTER_BUTTERWORTH_ALPHA,
        .fMecSpeedFilterButterworthBeta  = MEC_SPEED_FILTER_BUTTERWORTH_BETA,
    },
    .TempParameters =
    {
        .bMotorTempSensorType           = MOTOR_TEMP_SENSOR_TYPE,
        .bMotorTempMixed                = MOTOR_TEMP_MIXED,
        
        .hOverTempMotorThresholdC       = OV_TEMP_MOTOR_THRESHOLD_C,
        .hOverTempMotorHysteresisC      = OV_TEMP_MOTOR_HYSTERESIS_C,
        .hFoldbackMotorTempInterval     = FOLDBACK_MOTOR_TEMP_INTERVAL,
    },
    .HallSensorParameters =
    {
        .bHallSensorsPlacement          = HALL_SENSORS_PLACEMENT,
        .bHallAveragingFifoDepth        = HALL_AVERAGING_FIFO_DEPTH,
        .hHallPhaseShift                = HALL_PHASE_SHIFT,
        
        .bEnVibrationError              = EN_VIBRATION_ERROR,
    },
    .WheelSpeedSensorParameters =
    {
        .bWheelSpeedSensorNbrPerRotation = WHEEL_SPEED_SENSOR_NBR_PER_ROTATION,
    },
    .CurrentSpeedPID = 
    {
        .IqKpVsSpeedTable =
        {
            IQ_KP_VS_SPEED_1,
            IQ_KP_VS_SPEED_2,
        },
        .IqKiVsSpeedTable =
        {
            IQ_KI_VS_SPEED_1,   /* old PI = 50 parameter tunning for Vibration */ 
            IQ_KI_VS_SPEED_2,   /* old PI = 50 parameter tunning for Vibration */
        },
        .IdKpVsSpeedTable =
        {
            ID_KP_VS_SPEED_1,
            ID_KP_VS_SPEED_2,    
        },
        .IdKiVsSpeedTable =
        {
            ID_KI_VS_SPEED_1,     /* old PI = 50 parameter tunning for Vibration */
            ID_KI_VS_SPEED_2,     /* old PI = 50 parameter tunning for Vibration */
        },
    },
    .bAutotuneEnable = false,
};

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
  .hLowerOutputLimit    = 0, // -(int16_t)SPD_CTRL_MAX_TORQUE,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PIDHandle_t PIDIqHandleM1 =
{
    .hLowerOutputLimit       = -MAX_DUTY,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PIDHandle_t PIDIdHandleM1 =
{
    .hLowerOutputLimit       = -MAX_DUTY,
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
    .hVqdLowPassFilterBw    = M1_VQD_SW_FILTER_BW_FACTOR,
    .hVqdLowPassFilterBwLog = (uint16_t) M1_VQD_SW_FILTER_BW_FACTOR_LOG,           
};

/**
  * @brief  PI Motor Control control parameters Motor 1
  */
PIDHandle_t PIDMotorControlM1 =
{
    .hLowerOutputLimit       = -INT16_MAX,
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
        .FoldbackConfig = TRIM,
    },
        .FoldbackMotorTemperature =
    {
        .FoldbackConfig = TRIM,
    },
        .FoldbackHeatsinkTemperature =
    {
        .FoldbackConfig = TRIM,
    },
    .FoldbackDynamicMaxPower =
    {
        .FoldbackConfig = TRIM,
    },
    .FoldbackDynamicMaxTorque = 
    {
        .FoldbackConfig = SET_THRESHOLD,
    },
    .DynamicPowerHandle =
    {
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
    .hMaxBusCurrent =               (uint16_t)(DEFAULT_MAX_APPLICATION_CURRENT),
    .hMinAppPositiveMecSpeedUnit =    (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .hMinAppNegativeMecSpeedUnit =    (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
    .ModeDefault =                    DEFAULT_CONTROL_MODE,
    .fGainTorqueIdref =             GAIN_TORQUE_IDREF,
    .PISpeedLimit =
    {
        .hLowerOutputLimit    = 0,
        .hDefKdGain           = 0x0000U,
        .hKdDivisor           = 0x0000U,
        .hKdDivisorPOW2       = 0x0000U,
    },
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS, STM32G4xx
  */
PWMInsulCurrSensorFdbkHandle_t PWMInsulCurrSensorFdbkHandleM1 = 
{
    {
        .pFctGetPhaseCurrents              = &PWMInsulCurrSensorFdbk_GetPhaseCurrents,
        .pFctSwitchOffPwm                  = &PWMInsulCurrSensorFdbk_SwitchOffPWM,
        .pFctSwitchOnPwm                   = &PWMInsulCurrSensorFdbk_SwitchOnPWM,
        .pFctCurrReadingCalib              = &PWMInsulCurrSensorFdbk_CurrentReadingPolarization,
        .pFctTurnOnLowSides                = &PWMInsulCurrSensorFdbk_TurnOnLowSides,
        .pFctSetADCSampPointSectX          = &PWMInsulCurrSensorFdbk_WriteTIMRegisters,
        .pFctIsOverCurrentOccurred         = &PWMInsulCurrSensorFdbk_IsOverCurrentOccurred,
        #if OCDX_POEG == OCD1_POEG
            .pFctOCD2Occured                   = &PWMInsulCurrSensorFdbk_OCD2Occurred,
        #endif
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
        /*******************************************************/
        /*     Motor tuner pwm current feedback parameters     */
        /*******************************************************/
       #if AUTOTUNE_ENABLE
        .IaFilt = 0,
        .IbFilt = 0,
        .IcFilt = 0,
        .hPWMDeadtime = DEAD_TIME_COUNTS,
        .fCurrentConversionFactor =  (float)ADC_REFERENCE_VOLTAGE / (65536.0f * (float)AMPLIFICATION_GAIN),
        .fCycle2SecondConversionFactor = 1.0f / ((float)PWM_TIM_CLK_MHz * 1000000.0f),
       #endif

    },
    .hIaRaw = 0,
    .hIbRaw = 0,

    .bOverrunFlag = false,

    .wPhaseAOffset = 0,
    .wPhaseBOffset = 0,
    .hHalfPWMPeriod = PWM_PERIOD_CYCLES/2u,
    .bPolarizationCounter = 0,
    .bOCD1Flag = false,
    .bOCD2Flag = false,

    .pParamsStructure = &PWMICSParamsM1
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + PLL
  */
BemfObserverPllHandle_t BemfObserverPllM1 =
{
    .Super = {
        .bSpeedUnit                         =    SPEED_UNIT,
        .hMinReliableMecSpeedUnit          =    (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
        .bMaximumSpeedErrorsNumber         =    MEAS_ERRORS_BEFORE_FAULTS,
        .hMaxReliableMecAccelUnitP         =    65535,
        .hMeasurementFrequency             =    TF_REGULATION_RATE_SCALED,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .hC2                         =    C2,
    .hC4                         =    C4,
    .hF1                         =    F1,
    .hF2                         =    F2,
    .PIRegulator = {
        .hDefKdGain = 0x0000U,
        .hKdDivisor = 0x0000U,
        .hLowerOutputLimit = -INT16_MAX,
        .hKdDivisorPOW2       = 0x0000U,
    },
    .bSpeedBufferSizeUnit                =    STO_FIFO_DEPTH_UNIT,
    .bSpeedBufferSizeDpp                 =    STO_FIFO_DEPTH_DPP,
    .hVariancePercentage                 =    PERCENTAGE_FACTOR,
    .bSpeedValidationBandHigh              =    SPEED_BAND_UPPER_LIMIT,
    .bSpeedValidationBandLow              =    SPEED_BAND_LOWER_LIMIT,
    .hMinStartUpValidSpeed               =    OBS_MINIMUM_SPEED_UNIT,
    .bStartUpConsistThreshold            =    NB_CONSECUTIVE_TESTS,
    .bReliabilityHysteresys             =    OBS_MEAS_ERRORS_BEFORE_FAULTS,
    .bBemfConsistencyCheck               =    BEMF_CONSISTENCY_TOL,
    .bBemfConsistencyGain                =    BEMF_CONSISTENCY_GAIN,
    .hF1Log                              =    (uint16_t)F1_LOG,
    .hF2Log                              =    (uint16_t)F2_LOG,
    .hSpeedBufferSizeDppLog              =    (uint16_t)STO_FIFO_DEPTH_DPP_LOG,
    .hForcedDirection                   =  0x0000U
}; BemfObserverPllHandle_t *pBemfObserverPllM1 = &BemfObserverPllM1;

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */
HallPosSensorHandle_t HallPosSensorM1 =
{
    .Super = 
    {
        .hMinReliableMecSpeedUnit          =    (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
        .bMaximumSpeedErrorsNumber         =    MEAS_ERRORS_BEFORE_FAULTS,
        .hMaxReliableMecAccelUnitP         =    65535,
        .hMeasurementFrequency             =    TF_REGULATION_RATE_SCALED,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .TIMx                = HALL_POSITION_TIMER_HANDLE_ADDRESS,
    .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
    .wTIMClockFreq       = HALL_TIM_CLK,
    .bPWMFreqScaling      = PWM_FREQ_SCALING,
    .bHallMtpa            = HALL_MTPA,
    .H1PortPin = HALL_POSITION_HU_GPIO_PIN,
    .H2PortPin = HALL_POSITION_HV_GPIO_PIN,
    .H3PortPin = HALL_POSITION_HW_GPIO_PIN,
};

/**
  * temperature sensor parameters Motor 1
  */
NTCTempSensorHandle_t TempSensorMotorM1 =
{
    .TempRegConv =
    {
        .hChannel = MOTOR_TEMP_ANALOG_CHANNEL,
    },
    .hLowPassFilterBw        = M1_TEMP_SW_FILTER_BW_FACTOR,

    .pNTCLookupTable = &MotorNTCLookupTable,
};

/**
  * temperature sensor parameters Controller 1
  */
NTCTempSensorHandle_t TempSensorControllerM1 =
{
    .TempRegConv =
    {
        .hChannel = HEATSINK_TEMP_ANALOG_CHANNEL,
    },
    .hLowPassFilterBw        = M1_TEMP_SW_FILTER_BW_FACTOR,

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
    .hOverVoltageThreshold  =  OVERVOLTAGE_THRESHOLD_d,
    .hUnderVoltageThreshold =  DEFAULT_UNDERVOLTAGE_THRESHOLD_d,
    .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitationHandle_t CircleLimitationM1 =
{
    .hMaxModule          = MAX_MODULE,
    .hMaxVd                = (uint16_t)(MAX_MODULE * FW_VOLTAGE_REF / 1000),
    .CircleLimitTable = MMITABLE,
    .bStartIndex        = START_INDEX,
};

RotorPositionObserverHandle_t RotorPosObsM1 =
{
    .Super = {
    .hMinReliableMecSpeedUnit          =    (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =    MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =    65535,
    .hMeasurementFrequency             =    TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },

    .pHallSensor = &HallPosSensorM1,

    .hKpGainDef = ROTOR_POS_OBS_KP,
    .hKpDivisorPOW2 = (uint16_t) ROTOR_POS_OBS_KPDIV_LOG,

    .hKiGainDef = ROTOR_POS_OBS_KI,
    .hKiDivisorPOW2 = (uint16_t) ROTOR_POS_OBS_KIDIV_LOG,

    .hKdGainDef = ROTOR_POS_OBS_KD,
    .hKdDivisorPOW2 = (uint16_t) ROTOR_POS_OBS_KDDIV_LOG,

};
