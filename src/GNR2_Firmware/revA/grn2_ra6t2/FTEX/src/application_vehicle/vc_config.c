/**
  * @file    vc_config.c
  * @brief   This module declares global structures used by vehicle control application
  *
*/

#include "vc_config.h"
#include "gnr_parameters.h"
#include "board_hardware.h"
#include "vc_parameters.h"


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


/**@brief Throttle initializing Parameters.
 */
THRO_Handle_t ThrottleHandle =
{
	.Throttle_RegConv =
	{
			.hChannel = THROTTLE_ANALOG_CHANNEL,
	},
	.hParameters =
	{
		.hLowPassFilterBW1 = THROTTLE_LOWPASS_FILTER_BW1,
		.hLowPassFilterBW2 = THROTTLE_LOWPASS_FILTER_BW2,

		.hOffsetThrottle = THROTTLE_OFFSET_ADC2THROTTLE,
		.bSlopeThrottle = THROTTLE_SLOPE_ADC2THROTTLE,
		.bDivisorThrottle = THROTTLE_DIVISOR_ADC2THROTTLE,

		.hOffsetTorque = THROTTLE_OFFSET_THROTTLE2TORQUE,
		.bSlopeTorque = THROTTLE_SLOPE_THROTTLE2TORQUE,
		.bDivisorTorque = THROTTLE_DIVISOR_THROTTLE2TORQUE,

		.hDetectionThreshold = THROTTLE_DETECTION_THRESHOLD,
	}
};


/**@brief Pedal torque sensor initializing Parameters.
 */
PedalTorqSensorHandle_t PedalTorqueSensor =
{
    .PTS_RegConv =
    {
        .hChannel = PEDAL_TORQUE_SENSOR_ANALOG_CHANNEL,
    },
    .hParameters =
    {
        .hLowPassFilterBW1 = PTS_LOWPASS_FILTER_BW1,
        .hLowPassFilterBW2 = PTS_LOWPASS_FILTER_BW2,

        .hOffsetPTS = PTS_OFFSET_ADC2PTS,
        .bSlopePTS = PTS_SLOPE_ADC2PTS,
        .bDivisorPTS = PTS_DIVISOR_ADC2PTS,

        .hOffsetMT = PTS_OFFSET_PTS2TORQUE,
        .bSlopeMT = PTS_SLOPE_PTS2TORQUE,
        .bDivisorMT = PTS_DIVISOR_PTS2TORQUE,
    }
};

/**@brief Pulse Frequeny initializing Parameters.
 */
PulseFrequency_Handle_AGT_t PulseFreqHandle =
{
	.agt_start_measurement = false,
	.PulseFreqParam_AGT =
	{
		 .PF_AGT_Timer = &ag_timer0,
	}
};

/**@brief Pulse Frequeny initializing Parameters for GPT Timer.
 */
WheelFrequency_Handle_GPT_t PulseFreqHandle_Wheel =
{
	.gpt_start_measurement = false,
	.PulseFreqParam_GPT =
	{
		 .PF_GPT_Timer = &g_timer8,
	}
};

/**@brief Pedal assist initializing Parameters.
 */
PedalSpeedSens_Handle_t PedalAssistHandle = {
	.pTorque = &PedalTorqueSensor,
    .pSpulse = &PulseFreqHandle,
};


MS_Handle_t MotorSelectorHandle =
{
//	.wM1SelectPinNumber = M1SELECT_GPIO_PIN,
//	.wM2SelectPinNumber = M2SELECT_GPIO_PIN,

	.bIsInvertedLogic = false,
	.bMSEnable = MOTOR_SELECTOR_ENABLE,
};


PWREN_Handle_t PowerEnableHandle =
{
//	.wPinNumber = PWREN_GPIO_PIN,
	.bIsInvertedLogic = false,
	.bUsePowerLock = POWER_ENABLE_ENABLE,
};

PWRT_Handle_t PowertrainHandle =
{
	.sParameters.bUseMotorM1 = POWERTRAIN_USE_MOTOR1,
	.sParameters.bUseMotorM2 = POWERTRAIN_USE_MOTOR2,
	.sParameters.bDefaultMainMotor = POWERTRAIN_DEFAULT_MAIN_MOTOR,
	.sParameters.bMode = POWERTRAIN_DEFAULT_MODE,
	.sParameters.bCtrlType = POWERTRAIN_DEFAULT_CONTROL_TYPE,
	.sParameters.bM2TorqueInversion = POWERTRAIN_M2_TORQUE_INVERSION,
	.sParameters.hTorqueRampTimeUp = POWERTRAIN_THROTTLE_TORQUE_RAMPTIME_UP,
	.sParameters.hTorqueRampTimeDown = POWERTRAIN_THROTTLE_TORQUE_RAMPTIME_DOWN,
	.sParameters.hTorquePASRampTimeUp = POWERTRAIN_PAS_TORQUE_RAMPTIME_UP,
	.sParameters.hSpeedRampTimeUp = POWERTRAIN_THROTTLE_SPEED_RAMPTIME_UP,
	.sParameters.hSpeedRampTimeDown = POWERTRAIN_THROTTLE_SPEED_RAMPTIME_DOWN,
	.sParameters.hStartingThrottle = POWERTRAIN_START_THROTTLE_THRESHOLD,
	.sParameters.hStoppingThrottle = POWERTRAIN_STOP_THROTTLE_THRESHOLD,
	.sParameters.hStoppingSpeed = POWERTRAIN_STOP_SPEED_THRESHOLD,
	.sParameters.hPASMaxTorque = POWERTRAIN_PAS_MAX_TORQUE,
	.sParameters.hPASMaxSpeed = POWERTRAIN_PAS_MAX_SPEED,
	.sParameters.GearRatio = POWERTRAIN_MOTOR_GEARRATIO,
	.sParameters.bUseWheelSpeedSensor = POWERTRAIN_WHEEL_SPEED_SENSOR_ENABLE,
	.sParameters.bWheelSpreedRatio = POWERTRAIN_WHEEL_SPEED_SENSOR_PPR,
	.sParameters.hFaultManagementTimeout = POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT,

	.pMDI = &MDInterfaceHandle,
	.pThrottle = &ThrottleHandle,
	.pBrake = &BrakeHandle,
	.pMS = &MotorSelectorHandle,
	.pPWREN = &PowerEnableHandle,
	.pPAS = &PedalAssistHandle,

//	.sSpeedFoldback[M1] =
//	{
//		.bEnableFoldback = false,
//		.hStartValue = 200,
//		.hIntervalValue = 200,
//		.hEndValue = 400,
//		.hDefaultMaxTorque = 10000,
//	},
//	.sSpeedFoldback[M2] =
//	{
//		.bEnableFoldback = false,
//		.hStartValue = 200,
//		.hIntervalValue = 200,
//		.hEndValue = 400,
//		.hDefaultMaxTorque = 10000,
//	},

	.sHeatsinkTempFoldback[M1] =
	{
		.bEnableFoldback = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_ENABLE,
		.hStartValue = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_START,
		.hEndValue = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_END,
		.hDefaultMaxTorque = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_MAX_TORQUE,
	},
	.sHeatsinkTempFoldback[M2] =
	{
		.bEnableFoldback = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_ENABLE,
		.hStartValue = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_START,
		.hEndValue = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_END,
		.hDefaultMaxTorque = POWERTRAIN_HEATSINK_TEMP_FOLDBACK_MAX_TORQUE,
	},
};

VCI_Handle_t VCInterfaceHandle =
{
	.pStateMachine = &VCStateMachineHandle,
	.pPowertrain = &PowertrainHandle,
};
