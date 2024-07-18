/**
  * @file    vc_config.c
  * @brief   This module declares global structures used by vehicle control application
  *
*/

#include "vc_config.h"
#include "gnr_parameters.h"
#include "board_hardware.h"
#include "vc_parameters.h"
#include "hal_data.h"

// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop
        
MultipleDriveInterfaceHandle_t MDInterfaceHandle =
{
    0
};

VCSTM_Handle_t VCStateMachineHandle =
{
    .bVState = V_IDLE,
    .hVFaultNow = 0,
    .hVFaultOccurred = 0
};

BRK_Handle_t BrakeHandle =
{
    .wPinNumber = BRAKE_GPIO_PIN,
    .bIsInvertedLogic = true,
};

Light_Handle_t HeadLightHandle =
{
    .wPinNumber = FRONT_LIGHT_PIN,
    .bIsInvertedLogic = false,
    .bLightIsBlinking = false,
    .BlinkPeriode     = 0,
    .bDefaultLightState = POWERTRAIN_HEADLIGHT_DEFAULT,
    .bBlinkOnBrake = false,    
};

Light_Handle_t TailLightHandle =
{
    .wPinNumber = BACK_LIGHT_PIN,
    .bIsInvertedLogic = false,
    .bLightIsBlinking = false, 
    .BlinkPeriode     = 50,
    .bDefaultLightState = POWERTRAIN_TAILLIGHT_DEFAULT,
    .bBlinkOnBrake = REAR_LIGHT_BLINK_ON_BRAKE,    
};    

BatMonitor_Handle_t BatMonitorHandle = 
{
    .VBatMin = BATTERY_EMPTY_VOLT_X_100,                            // Values that represent a fully charged battery and an empty one
    .VBatMax = BATTERY_FULL_VOLT_X_100,                             // Set in the VC_parameters_xxxxx.h of each bike
    .LowBatSOC = BATTERY_SOC_LOW_PERCENT,                           // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
    .RechargedBatSOC = BATTERY_SOC_OK_PERCENT,                      // Battery SOC in % for which we clear the battery low flag
};


MS_Handle_t MotorSelectorHandle =
{
    .wM1SelectPinNumber = M1SELECT_GPIO_PIN,
    .wM2SelectPinNumber = M2SELECT_GPIO_PIN,

    .bIsInvertedLogic = false,
    .bMSEnable = MOTOR_SELECTOR_ENABLE,
};


PWREN_Handle_t PowerEnableHandle =
{
    .bInitalized = false,
    .bUsePowerLock = POWER_ENABLE_ENABLE,
    .bIsInvertedLogic = false,
    .bInitialPowerLockState = false,
    .bWakeUpSDOCommand = false,
    .bWakeUpCommandChecked = false,
    .bIotStoped = false,
    .bSlaveOff = false,
    .bGoingOff = false,
    .bSdoResponseReceived = false,
    .bSystemReady = false,
};


PAS_Handle_t PedalAssistHandle = 
{
    .sParameters.bMaxLevel = PAS_MAX_LEVEL,
    .sParameters.PasMaxSpeed = VEHICLE_TOP_SPEED_KMH,
    .sParameters.bTorqueGain[0] = PAS_0_TORQUE_GAIN,
    .sParameters.bTorqueGain[1] = PAS_1_TORQUE_GAIN,
    .sParameters.bTorqueGain[2] = PAS_2_TORQUE_GAIN,
    .sParameters.bTorqueGain[3] = PAS_3_TORQUE_GAIN,
    .sParameters.bTorqueGain[4] = PAS_4_TORQUE_GAIN,
    .sParameters.bTorqueGain[5] = PAS_5_TORQUE_GAIN,
    .sParameters.bTorqueGain[6] = PAS_6_TORQUE_GAIN,
    .sParameters.bTorqueGain[7] = PAS_7_TORQUE_GAIN,
    .sParameters.bTorqueGain[8] = PAS_8_TORQUE_GAIN,
    .sParameters.bTorqueGain[9] = PAS_9_TORQUE_GAIN,
    .sParameters.hMaxTorqueRatio = PAS_MAX_TORQUE_RATIO,
    .sParameters.WalkmodeOverThrottle = PAS_WALKMODE_OVER_THROTTLE,
    .sParameters.PASOverThrottle = PAS_OVER_THROTTLE,
    .sParameters.PASMaxSpeed[0] = PAS_LEVEL_SPEED_0,
    .sParameters.PASMaxSpeed[1] = PAS_LEVEL_SPEED_1,
    .sParameters.PASMaxSpeed[2] = PAS_LEVEL_SPEED_2,
    .sParameters.PASMaxSpeed[3] = PAS_LEVEL_SPEED_3,
    .sParameters.PASMaxSpeed[4] = PAS_LEVEL_SPEED_4,
    .sParameters.PASMaxSpeed[5] = PAS_LEVEL_SPEED_5,
    .sParameters.PASMaxSpeed[6] = PAS_LEVEL_SPEED_6,
    .sParameters.PASMaxSpeed[7] = PAS_LEVEL_SPEED_7,
    .sParameters.PASMaxSpeed[8] = PAS_LEVEL_SPEED_8,
    .sParameters.PASMaxSpeed[9] = PAS_LEVEL_SPEED_9,
    .sParameters.walkModeTorqueRatio = PAS_WALK_POWER_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[0] = PAS_0_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[1] = PAS_1_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[2] = PAS_2_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[3] = PAS_3_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[4] = PAS_4_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[5] = PAS_5_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[6] = PAS_6_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[7] = PAS_7_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[8] = PAS_8_MIN_TORQUE_PERCENT,
    .sParameters.PASMinTorqRatiosInPercentage[9] = PAS_9_MIN_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[0] = PAS_0_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[1] = PAS_1_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[2] = PAS_2_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[3] = PAS_3_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[4] = PAS_4_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[5] = PAS_5_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[6] = PAS_6_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[7] = PAS_7_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[8] = PAS_8_MAX_TORQUE_PERCENT,
    .sParameters.PASMaxTorqRatiosInPercentage[9] = PAS_9_MAX_TORQUE_PERCENT,

    .sParameters.PASSpeedThresholds[0] = STARTUP_PAS_SPEED_THRESHOLD,
    .sParameters.PASSpeedThresholds[1] = RUNTIME_PAS_SPEED_THRESHOLD,
    .sParameters.PASTorqueScalingPedalRPMActivated = TORQUE_SCALING_PEDAL_RPM,
    .sParameters.PASTorqueScalingPedalRPMParameters[0] = MIN_RPM_SCALING,
    .sParameters.PASTorqueScalingPedalRPMParameters[1] = MAX_RPM_SCALING,
    .sParameters.PASTorqueScalingPedalRPMParameters[2] = GAIN_AT_MIN_RPM,
    .sParameters.PASTorqueScalingPedalRPMParameters[3] = GAIN_AT_MAX_RPM,
    .sParameters.rampType = PAS_RAMP_SELECTION,
    
    .bPASPowerEnable = false,
    .bPASCadenceRunningOverride = false,
    .bPASTorqueRunningOverride = false,
    .cadenceAndOrTorqueFlag = CADENCE_AND_OR_TORQUE,

    .bStartupPasAlgorithm = PAS_DETECTIONSTARTUP_ALGORITHM,
    .bRunningPasAlgorithm = PAS_DETECTIONRUNNING_ALGORITHM,
    
    .torqueSensorIssueTimer = 0,
    .bTorqueSensorIssue = false,
};

/**@brief Throttle initializing Parameters.
 */
ThrottleHandle_t ThrottleHandle =
{
    .Throttle_RegConv =
    {
            .hChannel = THROTTLE_ANALOG_CHANNEL,
    },
    .ThrottleFilter = 
    {
        .pIIRFAInstance = NULL, // NULL to apply software filtering, no hardware accelerator
    },
    .BlockOffThrottle = THROTTLE_BLOCK_OFF,
    .hParameters =
    {
        .fFilterAlpha = THROTTLE_FILTER_ALPHA,
        .fFilterBeta  = THROTTLE_FILTER_BETA,

        .hOffsetThrottle  = THROTTLE_OFFSET_ADC2THROTTLE,
        
        .hMaxThrottle = THROTTLE_MAX_ADC2THROTTLE,
        
        .hOffsetTorque  = THROTTLE_OFFSET_THROTTLE2TORQUE,
        
        .hOffsetSpeed  = THROTTLE_OFFSET_THROTTLE2SPEED,

        .hDetectionThreshold = THROTTLE_DETECTION_THRESHOLD,
        
        .DefaultMaxThrottleSpeedKMH = THROTTLE_TOP_SPEED,
    },
};

PWRT_Handle_t PowertrainHandle =
{
    .sParameters.bUseMotorM1 = POWERTRAIN_USE_MOTOR1,
    .sParameters.bUseMotorM2 = POWERTRAIN_USE_MOTOR2,
    .sParameters.bDefaultMainMotor = POWERTRAIN_DEFAULT_MAIN_MOTOR,
    .sParameters.bMode = POWERTRAIN_DEFAULT_MODE,
    .sParameters.bCtrlType = POWERTRAIN_DEFAULT_CONTROL_TYPE,
    .sParameters.bM2TorqueInversion = POWERTRAIN_M2_TORQUE_INVERSION,
    .sParameters.hStartingThrottle = POWERTRAIN_START_THROTTLE_THRESHOLD,
    .sParameters.hStoppingThrottle = POWERTRAIN_STOP_THROTTLE_THRESHOLD,
    .sParameters.hStoppingSpeed = POWERTRAIN_STOP_SPEED_THRESHOLD,
    
    .sParameters.bPAS0DisableThrottle = POWERTRAIN_DISABLE_THROTTLE_PAS_0,
    
    .sParameters.hFaultManagementTimeout = POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT,
    .sParameters.bEnableSpeedLimit = ENABLE_SPEED_LIMIT,
    .sParameters.VehicleMaxSpeed = VEHICLE_TOP_SPEED_KMH,    
    .pMDI = &MDInterfaceHandle,
    .pThrottle = &ThrottleHandle,
    .pBrake = &BrakeHandle,
    .pHeadLight = &HeadLightHandle,
    .pTailLight = &TailLightHandle,
    .pBatMonitorHandle = &BatMonitorHandle,
    .pMS = &MotorSelectorHandle,
    .pPWREN = &PowerEnableHandle,
    .pPAS = &PedalAssistHandle,
    .powertrainLockStatus = false    
};

VCI_Handle_t VCInterfaceHandle =
{
    .pStateMachine = &VCStateMachineHandle,
    .pPowertrain = &PowertrainHandle,
    .pFirmwareUpdateDomainObj = &bObjFirmwareUpdateDomain,
};


/**@brief MC setup initializing parameters.
 */
MC_Setup_t MCSetup = 
{
    /**@brief Battery power initializing parameters.
    */
    .BatteryPowerSetup =
    {
        .bPowerLimitRef                 = POWER_LIMIT_REF,                // Defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
        .hMaxApplicationPositivePower   = MAX_APPLICATION_POSITIVE_POWER, // Maximum power in watts that drive can push to the motor
        .hMaxApplicationNegativePower   = MAX_APPLICATION_NEGATIVE_POWER, // Maximum power in watts that drive can accept from the motor
        .hMaxApplicationCurrent         = MAX_APPLICATION_CURRENT,        // Maximum battery current in amps that drive can accept from the motor
        
        .bEnableLVTorqueLimit           = ENABLE_LV_TORQUE_LIMIT,
        .hLowVoltageThresholdPercentage = LOW_VOLTAGE_THRESHOLD_PERCENTAGE,
        .hLowBatteryTorque              = LOW_BATTERY_TORQUE,
        
        .bEnableMaxPowerLimit           = ENABLE_MAX_POWER_LIMIT,
        .wMaxTimeBMSTolerant            = MAX_TIME_BMS_TOLERANT,
        .hMaxPowerLimitTimeout          = MAX_POWER_LIMIT_TIMEOUT,
        .hMaxBMSPositivePower           = MAX_BMS_POSITIVE_POWER,
        .hMaxBMSContinuousCurrent       = MAX_BMS_CONTINUOUS_CURRENT,
        
        .hUndervoltageThreshold         = UD_VOLTAGE_THRESHOLD_BATT_V,
    },
    
    .bEnSpeedLimit                      = ENABLE_SPEED_LIMIT,
};
