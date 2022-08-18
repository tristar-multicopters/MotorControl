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
		.fFilterBeta = THROTTLE_FILTER_BETA,

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
    }
};

///**@brief Pulse Frequency initializing Parameters.
// */
//PulseFrequency_Handle_AGT_t PulseFreqHandle =
//{
//	.agt_start_measurement = false,
//	.PulseFreqParam_AGT =
//	{
//		.PF_AGT_Timer = PEDAL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
//	}
//};

///**@brief Pedal assist initializing Parameters.
// */
//PedalSpeedSensorHandle_t PedalAssistHandle = {
//	.pTorque = &PedalTorqueSensor,
//    .pSpulse = &PulseFreqHandle,
//};

///**@brief Pulse Frequency initializing Parameters for GPT Timer.
// */
//WheelFrequency_Handle_GPT_t PulseFreqHandle_Wheel =
//{
//	.gpt_start_measurement = false,
//	.PulseFreqParam_GPT =
//	{
//		 .PF_GPT_Timer = WHEEL_SPEED_SENSOR_TIMER_HANDLE_ADDRESS,
//	}
//};

//WheelSpeedSens_Handle_t WheelSpeedHandle =
//{
//    .wSpulse = &PulseFreqHandle_Wheel,
//};



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
	.sParameters.hStartingThrottle = POWERTRAIN_START_THROTTLE_THRESHOLD,
	.sParameters.hStoppingThrottle = POWERTRAIN_STOP_THROTTLE_THRESHOLD,
	.sParameters.hStoppingSpeed = POWERTRAIN_STOP_SPEED_THRESHOLD,
	.sParameters.hPASMaxTorque = POWERTRAIN_PAS_MAX_TORQUE,
	.sParameters.hPASMaxSpeed = POWERTRAIN_PAS_MAX_SPEED,
	.sParameters.MotorToHubGearRatio = POWERTRAIN_MOTOR_GEARRATIO,
	.sParameters.hFaultManagementTimeout = POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT,

	.pMDI = &MDInterfaceHandle,
	.pThrottle = &ThrottleHandle,
	.pBrake = &BrakeHandle,
	.pMS = &MotorSelectorHandle,
	.pPWREN = &PowerEnableHandle,

};

VCI_Handle_t VCInterfaceHandle =
{
	.pStateMachine = &VCStateMachineHandle,
	.pPowertrain = &PowertrainHandle,
};
