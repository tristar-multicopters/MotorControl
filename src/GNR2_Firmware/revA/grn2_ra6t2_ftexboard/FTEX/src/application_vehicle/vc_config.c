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


/*  these are just temporal definitiions needed by speed control te be tested with throttle
    since we have no clear plan for using speed control with trottle at the moment,
    and to prevent any exception (dividing by zero) or error, the definitions kept here alive
    until someone update the code with clear and clean revise  */
#if POWERTRAIN_DEFAULT_CONTROL_TYPE == SPEED_CTRL
        #define THROTTLE_OFFSET_THROTTLE2SPEED         400 //400  is tested value for R48 // Offset for throttle to speed linear transformation 
        #define THROTTLE_SLOPE_THROTTLE2SPEED          20 //20   is tested value for R48 // Slope for throttle to speed linear transformation
        #define THROTTLE_DIVISOR_THROTTLE2SPEED        140 //140  is tested value for R48 // Divisor for throttle to speed linear transformation
#elif  POWERTRAIN_DEFAULT_CONTROL_TYPE == TORQUE_CTRL
        #define THROTTLE_OFFSET_THROTTLE2SPEED         1 //400  is tested value for R48 // Offset for throttle to speed linear transformation 
        #define THROTTLE_SLOPE_THROTTLE2SPEED          1 //20   is tested value for R48 // Slope for throttle to speed linear transformation
        #define THROTTLE_DIVISOR_THROTTLE2SPEED        1 //140  is tested value for R48 // Divisor for throttle to speed linear transformation
#endif
        
/* end of temperory defined values */

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
    .bLightStateLocked = POWERTRAIN_HEADLIGHT_LOCKED,
    .bDefaultLightState = POWERTRAIN_HEADLIGHT_DEFAULT,     
};

Light_Handle_t TailLightHandle =
{
    .wPinNumber = BACK_LIGHT_PIN,
    .bIsInvertedLogic = false,
    .bLightIsBlinking = false, 
    .BlinkPeriode     = 50,
    .bLightStateLocked = POWERTRAIN_TAILLIGHT_LOCKED,
    .bDefaultLightState = POWERTRAIN_TAILLIGHT_DEFAULT,   
};    

BatMonitor_Handle_t BatMonitorHandle = 
{
    .VBatMin = BATTERY_EMPTY_VOLT,  // Values that represent a fully charged battery and an empty one
    .VBatMax = BATTERY_FULL_VOLT,   // Set in the VC_parameters_xxxxx.h of each bike
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
        .hLowPassFilterBW1 = PTS_FILTER_BW1,
        .hLowPassFilterBW2 = PTS_FILTER_BW2,
        .PasMaxOutputTorque = PAS_MAX_TORQUE,
    }
};

/**@brief Pulse Frequency initializing Parameters.
 */
PulseFrequencyHandle_t PulseFreqHandlePedal =
{    
    .TimerType = AGT_TIMER,    
    .start_measurement = false,    
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
    .start_measurement = false,    
    .PulseFreqParam =
    {
        .PF_Timer = WHEEL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
    }
};

/**@brief Pedal assist initializing Parameters.
 */
PedalSpeedSensorHandle_t PedalSpeedSensorHandle = {
    .pPulseFrequency = &PulseFreqHandlePedal,
};


WheelSpeedSensorHandle_t WheelSpeedHandle =
{
    .pPulseFrequency = &PulseFreqHandleWheel,
    .bPulsePerRotation = WHEEL_SPEED_SENSOR_NBR_PER_ROTATION,
    .bSpeedslowDetect = WHEEL_SPEED_SLOW_LOOP_DETECT,
    .bSlowDetectCountValue = WHEEL_SPEED_SLOW_LOOP_COUNT, 
    .bSpeedslowDetectCorrection = WHEEL_SPEED_SENSOR_CORRECTION_FACTOR
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

Foldback_Handle_t SpeedFoldbackVehicleConfig =
{
    .hDecreasingEndValue = POWERTRAIN_FOLDBACK_SPEED_END,
    .hDecreasingRange = POWERTRAIN_FOLDBACK_SPEED_RANGE,
    .hDecreasingInterval = POWERTRAIN_FOLDBACK_SPEED_INTERVAL,
    .hDefaultOutputLimitHigh = POWERTRAIN_MAX_MOTOR_TORQUE,
    .hDefaultOutputLimitLow = 0,
    .FoldbackConfig = TRIM,
    .bEnableFoldback = true,
    .hSlowStartBandwidth = FOLDBACK_SLOW_START_BANDWIDTH,
    .hSlowStopBandwidth = FOLDBACK_SLOW_STOP_BANDWIDTH,
    .wSlowStartTimeout =  FOLDBACK_TIMEOUT,
};

Foldback_Handle_t SpeedFoldbackThrottle =
{
    .hDecreasingEndValue = 300, 
    .hDecreasingRange    = 300,
    .hDecreasingInterval = 30, 
    .hDefaultOutputLimitHigh = POWERTRAIN_MAX_MOTOR_TORQUE,
    .hDefaultOutputLimitLow = 0,
    .FoldbackConfig = TRIM,
    .bEnableFoldback = true,
    .hSlowStartBandwidth = FOLDBACK_SLOW_START_BANDWIDTH,
    .hSlowStopBandwidth = FOLDBACK_SLOW_STOP_BANDWIDTH,
    .wSlowStartTimeout =  FOLDBACK_TIMEOUT,
};


PAS_Handle_t PedalAssistHandle = 
{
    .sParameters.hPASMaxTorque = PAS_MAX_TORQUE,
    .sParameters.hPASMaxSpeed = PAS_MAX_SPEED,
    .sParameters.hPASMaxKmSpeed = PAS_MAX_KM_SPEED,
    .sParameters.bMaxLevel = PAS_MAX_LEVEL,
    .sParameters.bTorqueGain = PAS_TORQUE_GAIN,
    .sParameters.hMaxTorqueRatio = PAS_MAX_TORQUE_RATIO,
    .sParameters.hMaxSpeedRatio = PAS_MAX_SPEED_RATIO,
    .sParameters.bPASCountSafe = PAS_MIN_PEDAL_COUNT_SAFE,
    .sParameters.bPASCountActivation = PAS_SLOW_PEDAL_COUNT,
    .sParameters.WalkmodeOverThrottle = PAS_WALKMODE_OVER_THROTTLE,
    .sParameters.UseCadenceSpeedLimit = PAS_CADENCE_USE_SPEED_LIMIT,
    .sParameters.PASTorqueRatiosInPercentage[0] = PAS_0_POWER_PERCENT,
    .sParameters.PASTorqueRatiosInPercentage[1] = PAS_1_POWER_PERCENT,
    .sParameters.PASTorqueRatiosInPercentage[2] = PAS_2_POWER_PERCENT,
    .sParameters.PASTorqueRatiosInPercentage[3] = PAS_3_POWER_PERCENT,
    .sParameters.PASTorqueRatiosInPercentage[4] = PAS_4_POWER_PERCENT,
    .sParameters.PASTorqueRatiosInPercentage[5] = PAS_5_POWER_PERCENT,
    .sParameters.walkModeTorqueRatio = PAS_WALK_POWER_PERCENT,
    .bCurrentPasAlgorithm = PAS_ALGORITHM,
    .pPSS = &PedalSpeedSensorHandle,
    .pPTS = &PedalTorqueSensorHandle,
    .pWSS = &WheelSpeedHandle,       
    .SpeedFoldbackVehiclePAS = &SpeedFoldbackVehicleConfig,

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
    .SpeedFoldbackVehicleThrottle = &SpeedFoldbackThrottle,
    .hParameters =
    {
        .fFilterAlpha = THROTTLE_FILTER_ALPHA,
        .fFilterBeta  = THROTTLE_FILTER_BETA,

        .hOffsetThrottle  = THROTTLE_OFFSET_ADC2THROTTLE,
        
        .hMaxThrottle = THROTTLE_MAX_ADC2THROTTLE,
        
        .hOffsetTorque  = THROTTLE_OFFSET_THROTTLE2TORQUE,
        
        .hOffsetSpeed  = THROTTLE_OFFSET_THROTTLE2SPEED,
        .bSlopeSpeed   = THROTTLE_SLOPE_THROTTLE2SPEED,
        .bDivisorSpeed = THROTTLE_DIVISOR_THROTTLE2SPEED,

        .hDetectionThreshold = THROTTLE_DETECTION_THRESHOLD,
        
        .MaxSafeThrottleSpeedKMH    = THROTTLE_MAX_SAFE_SPEED_KMH,
        .DefaultMaxThrottleSpeedKMH = THROTTLE_DEFAULT_MAX_SPEED_KMH,
        .ThrottleDecreasingRange    = THROTTLE_SPEED_DECREASING_RANGE,        
    }
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
    
    .sParameters.MotorToHubGearRatio = POWERTRAIN_MOTOR_GEARRATIO,
    .sParameters.hFaultManagementTimeout = POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT,
    
#ifdef VEHICLE_SPEED_KMH_POWER_CUTOFF
    .sParameters.bTopSpeedPowerCutoffEnable = true,
    .sParameters.TopSpeedKMHCutoff = VEHICLE_SPEED_KMH_POWER_CUTOFF,
#else
    .sParameters.bTopSpeedPowerCutoffEnable = false,
    .sParameters.TopSpeedKMHCutoff = 100,   // we set this to an unachievable speed to ensure 
                                            // if it is accidently used it will not limit the power      
#endif    
    
    .SpeedFoldbackVehicle = &SpeedFoldbackVehicleConfig,

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
