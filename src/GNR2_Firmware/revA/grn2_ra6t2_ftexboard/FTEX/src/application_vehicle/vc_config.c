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
    .VBatMin = BATTERY_EMPTY_VOLT_X_100,  // Values that represent a fully charged battery and an empty one
    .VBatMax = BATTERY_FULL_VOLT_X_100,   // Set in the VC_parameters_xxxxx.h of each bike
    .LowBatSOC = BATTERY_SOC_LOW_PERCENT,        // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
    .RechargedBatSOC = BATTERY_SOC_OK_PERCENT,   // Battery SOC in % for which we clear the battery low flag
};


/**@brief Pedal torque sensor initializing Parameters.
 */
PedalTorqSensorHandle_t PedalTorqueSensorHandle =
{
    .PTSRegConv =
    {
        .hChannel = PEDAL_TORQUE_SENSOR_ANALOG_CHANNEL,
    },
    .TorqSensorFilter = 
    {
        .pIIRFAInstance = NULL, // NULL to apply software filtering, no hardware accelerator
    },
    .hParameters =
    {
        .fFilterAlpha = PTS_FILTER_ALPHA,
        .fFilterBeta = PTS_FILTER_BETA,

        .hOffsetPTS = PTS_OFFSET_ADC2PTS,
        
        .hOffsetMTStartup      = PTS_OFFSET_PTS2TORQUE_STARTUP,  
        .hStartupOffsetMTSpeedKMH = PTS_OFFSET_STARTUP_SPEED_KMH,
        .hOffsetMT = PTS_OFFSET_PTS2TORQUE,
        .hOffsetMTSafety = PTS_OFFSET_PTS2TORQUE_SAFETY,
       
        .hMax = PTS_MAX_PTSVALUE,
        .hFilterSpeed[0] = PTS_SPEED_FILTER_1,
        .hFilterSpeed[1] = PTS_SPEED_FILTER_2,
        .hLowPassFilterBW1[0] = PTS_FILTER_BW1_1,
        .hLowPassFilterBW2[0] = PTS_FILTER_BW2_1,
        .hLowPassFilterBW1[1] = PTS_FILTER_BW1_2,
        .hLowPassFilterBW2[1] = PTS_FILTER_BW2_2,
        .hLowPassFilterBW1[2] = PTS_FILTER_BW1_3,
        .hLowPassFilterBW2[2] = PTS_FILTER_BW2_3,
        .PasMaxOutputTorque = PAS_MAX_TORQUE,
    }
};

/**@brief Pulse Frequency initializing Parameters.
 */
PulseFrequencyHandle_t PulseFreqHandlePedal =
{    
    .TimerType = AGT_TIMER,    
    .measuring = false, 
    .hNumberOfPulse = 0,        
    .PulseFreqParam =
    {
        .PF_Timer = PEDAL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
    }
};

/**@brief Pulse Frequency initializing Parameters.
 */
PulseFrequencyHandle_t PulseFreqHandleWheel =
{        
    .TimerType = GPT_TIMER,
    .measuring = false, 
    .wCaptureCount = 0,
    .wCaptureOverflow = 0,    
    .PulseFreqParam =
    {
        .PF_Timer = WHEEL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
    }
};

/**@brief Pedal assist initializing Parameters.
 */
PedalSpeedSensorHandle_t PedalSpeedSensorHandle = {
    .pPulseFrequency = &PulseFreqHandlePedal,
    .hPedalSpeedSens_MinPulseStartup = PEDALSPEEDSENSOR_MIN_PULSE_STARTUP,
    .wPedalSpeedSens_WindowsStartup = PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS,
    .hPedalSpeedSens_MinPulseRunning = PEDALSPEEDSENSOR_MIN_PULSE_RUNNING,
    .wPedalSpeedSens_WindowsRunning = PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS,
    .bPedalSpeedSens_ResetWindowsFlag = false,
};


WheelSpeedSensorHandle_t WheelSpeedHandle =
{
    .pPulseFrequency = &PulseFreqHandleWheel,
    .bWheelSpeed_PulsePerRotation = WHEEL_SPEED_SENSOR_NBR_PER_ROTATION,
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
    .sParameters.hPASMaxTorque = PAS_MAX_TORQUE,
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

    .bPasPowerAlgorithm = PAS_POWER_ALGORITHM,
    .bStartupPasAlgorithm = PAS_DETECTIONSTARTUP_ALGORITHM,
    .bRunningPasAlgorithm = PAS_DETECTIONRUNNING_ALGORITHM,
    .pPSS = &PedalSpeedSensorHandle,
    .pPTS = &PedalTorqueSensorHandle,
    .pWSS = &WheelSpeedHandle,       
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
    .hParameters =
    {
        .fFilterAlpha = THROTTLE_FILTER_ALPHA,
        .fFilterBeta  = THROTTLE_FILTER_BETA,

        .hOffsetThrottle  = THROTTLE_OFFSET_ADC2THROTTLE,
        
        .hMaxThrottle = THROTTLE_MAX_ADC2THROTTLE,
        
        .hOffsetTorque  = THROTTLE_OFFSET_THROTTLE2TORQUE,
        
        .hOffsetSpeed  = THROTTLE_OFFSET_THROTTLE2SPEED,

        .hDetectionThreshold = THROTTLE_DETECTION_THRESHOLD,
        
        .DefaultMaxThrottleSpeedKMH = VEHICLE_TOP_SPEED_KMH,
        .ThrottleMaxTorque = POWERTRAIN_MAX_MOTOR_TORQUE,
    },
    .BlockOffThrottle = THROTTLE_BLOCK_OFF,
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
    .sParameters.VehicleMaxSpeed = VEHICLE_TOP_SPEED_KMH,    
    .sParameters.TorqueSpeedLimitGain = TORQUE_SPEED_LIMIT_GAIN,
    .pMDI = &MDInterfaceHandle,
    .pThrottle = &ThrottleHandle,
    .pBrake = &BrakeHandle,
    .pHeadLight = &HeadLightHandle,
    .pTailLight = &TailLightHandle,
    .pBatMonitorHandle = &BatMonitorHandle,
    .pMS = &MotorSelectorHandle,
    .pPWREN = &PowerEnableHandle,
    .pPAS = &PedalAssistHandle,    
};

VCI_Handle_t VCInterfaceHandle =
{
    .pStateMachine = &VCStateMachineHandle,
    .pPowertrain = &PowertrainHandle,
    .pFirmwareUpdateDomainObj = &bObjFirmwareUpdateDomain,
};
