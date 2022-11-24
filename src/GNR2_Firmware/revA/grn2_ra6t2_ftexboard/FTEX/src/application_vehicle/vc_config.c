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
};

Light_Handle_t TailLightHandle =
{
    .wPinNumber = BACK_LIGHT_PIN,
    .bIsInvertedLogic = false,
    .bLightIsBlinking = false, 
    .BlinkPeriode     = 50,    
};    

BatMonitor_Handle_t BatMonitorHandle = 
{
    .VBatMin = BATTERY_EMPTY_VOLT,  // Values that represent a fully charged battery and an empty one
    .VBatMax = BATTERY_FULL_VOLT,   // Set in the VC_parameters_xxxxx.h of each bike
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
		.bSlopeThrottle   = THROTTLE_SLOPE_ADC2THROTTLE,
		.bDivisorThrottle = THROTTLE_DIVISOR_ADC2THROTTLE,

		.hOffsetTorque  = THROTTLE_OFFSET_THROTTLE2TORQUE,
		.bSlopeTorque   = THROTTLE_SLOPE_THROTTLE2TORQUE,
		.bDivisorTorque = THROTTLE_DIVISOR_THROTTLE2TORQUE,

		.hDetectionThreshold = THROTTLE_DETECTION_THRESHOLD,
	}
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
        .bSlopePTS = PTS_SLOPE_ADC2PTS,
        .bDivisorPTS = PTS_DIVISOR_ADC2PTS,

        .hOffsetMT = PTS_OFFSET_PTS2TORQUE,
        .bSlopeMT = PTS_SLOPE_PTS2TORQUE,
        .bDivisorMT = PTS_DIVISOR_PTS2TORQUE,
        .hMax = PTS_MAX_PTSVALUE,
        .hLowPassFilterBW1 = PTS_FILTER_BW1,
        .hLowPassFilterBW2 = PTS_FILTER_BW2,
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
	.bIsInvertedLogic = false,
	.bUsePowerLock = POWER_ENABLE_ENABLE,
};

Foldback_Handle_t SpeedFoldbackStartupDualMotorConfig =
{
        .hDecreasingEndValue = POWERTRAIN_DUAL_MOTOR_STARTUP_SPEED_END,
        .hDecreasingRange = POWERTRAIN_DUAL_MOTOR_STARTUP_SPEED_INTERVAL,
        .hDecreasingInterval = POWERTRAIN_DUAL_MOTOR_SPEED_INTERVAL,
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
    .sParameters.bCoeffLevel = PAS_LEVEL_COEFF,
    .sParameters.hMaxTorqueRatio = PAS_MAX_TORQUE_RATIO,
    .sParameters.hMaxSpeedRatio = PAS_MAX_SPEED_RATIO,
    .sParameters.bTorqueSensorUse = PAS_TORQUE_USE,
	.sParameters.bPASCountSafe = PAS_MIN_PEDAL_COUNT_SAFE,
    .sParameters.WalkmodeOverThrottle = PAS_WALKMODE_OVER_THROTTLE,
    .pPSS = &PedalSpeedSensorHandle,
    .pPTS = &PedalTorqueSensorHandle,
    .pWSS = &WheelSpeedHandle,	   
    .SpeedFoldbackStartupDualMotorPAS = &SpeedFoldbackStartupDualMotorConfig,

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
    .sParameters.bEnableDualMotorStartup = POWERTRAIN_DUAL_MOTOR_STARTUP_ENABLE,

        
    .SpeedFoldbackStartupDualMotor = &SpeedFoldbackStartupDualMotorConfig,

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
};
