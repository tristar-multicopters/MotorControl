/**
  * @file    vc_config.c
  * @brief   This module declares global structures used by vehicle control application
  *
*/

#include "vc_config.h"


//*******************************************************************************
//	Vehicle
//*******************************************************************************

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
//	.wPinNumber = BRAKE_GPIO_PIN,
	.bIsInvertedLogic = true,
};

/**@brief Pedal torque sensor initializing Parameters.
 */
PedalTorqSensorHandle_t PedalTorqueSensor =
{
    .PTS_RegConv =
    {
        .regADC = &g_adc,
        .channel = ADC_CHANNEL_20,
        .group = GROUP_1,
    },
    .hParam =
    {	
        .hLowPassFilterBW1 = 5,
        .hLowPassFilterBW2 = 25,

        .hOffsetPTS = 12500,
        .bSlopePTS = 5,			
        .bDivisorPTS = 4,

        .hOffsetMT = 20000, /* Offset to launch the torque sensing */
        .bSlopeMT = -7,
        .bDivisorMT = 2,
    }

};
/**@brief Pulse Frequeny initializing Parameters.
 */
PF_Handle_AGT_t PulseFreqHandle =
{
	.agt_start_measurement = false,
	.PF_Param_AGT =
	{
		 .PF_AGT_Timer = &ag_timer0,
	}
};

/**@brief Pedal assist initializing Parameters.
 */
PAS_Handle_t PedalAssistHandle = {
	.pTorque = &PedalTorqueSensor,
    .pSpulse = &PulseFreqHandle,
};

/**@brief Throttle initializing Parameters.
 */
THRO_Handle_t ThrottleHandle =
{
    .Throttle_RegConv =
    {
        .regADC = &g_adc,
        .channel = ADC_CHANNEL_21, //TODO: update to selected channel
        .group = GROUP_1,
    },
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
		
		.hOffsetThrottle = 10000,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -7,
		.bDivisorTorque = 25,
	
		.hDetectionThreshold = 1000,
	}
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 10000,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -7,
		.bDivisorTorque = 45,
		
		.hDetectionThreshold = 1000,
	}
		#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 10000,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -14,
		.bDivisorTorque = 48,
		
		.hDetectionThreshold = 1000,
	}
	
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 10000,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = 8,
		.bDivisorTorque = 32,
	
		.hDetectionThreshold = 1000,
	}
	#else
	.hParam =
	{
		.hLowPassFilterBW1 = 16,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 10000,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -7,
		.bDivisorTorque = 25,
		
		.hDetectionThreshold = 1000,
	}
	#endif
};


MS_Handle_t MotorSelectorHandle = 
{
//	.wM1SelectPinNumber = M1SELECT_GPIO_PIN,
//	.wM2SelectPinNumber = M2SELECT_GPIO_PIN,
	
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bIsInvertedLogic = false,
	.bMSEnable = true,
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bIsInvertedLogic = false,
	.bMSEnable = false,
		#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bIsInvertedLogic = false,
	.bMSEnable = false,
		#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.bIsInvertedLogic = false,
	.bMSEnable = false,
	#else
	.bIsInvertedLogic = false,
	.bMSEnable = true,
	#endif
};


PWREN_Handle_t PowerEnableHandle = {
//	.wPinNumber = PWREN_GPIO_PIN,
	.bIsInvertedLogic = false,
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bUsePowerLock = true,
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bUsePowerLock = false,
	#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bUsePowerLock = true,
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.bUsePowerLock = false,
	#else
	.bUsePowerLock = true;
	#endif
};

PWRT_Handle_t PowertrainHandle = 
{	
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.sParameters.bUseMotorM1 = true,
	.sParameters.bUseMotorM2 = true,
	.sParameters.bDefaultMainMotor = M1,
	.sParameters.bMode = DUAL_MOTOR,
	.sParameters.bCtrlType = TORQUE_CTRL,
	.sParameters.bM2TorqueInversion = false,
	.sParameters.hTorqueRampTimeUp = 200,
	.sParameters.hTorqueRampTimeDown = 50,
	.sParameters.hSpeedRampTimeUp = 200,
	.sParameters.hSpeedRampTimeDown = 50,
	.sParameters.hStartingThrottle = 1000,
	.sParameters.hStoppingThrottle = 500,
	.sParameters.hStoppingSpeed = 0,
	.sParameters.hPASMaxTorque = -10000,
	.sParameters.hPASMaxSpeed = 500,
  .sParameters.GearRatio = 0x00010001, //Ratio is unknown so 1/1 assumed
	.sParameters.hFaultManagementTimeout = 25, // Timer of 500ms for clear OC, SF and SU faults (20ms * 25)
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.sParameters.bUseMotorM1 = true,
	.sParameters.bUseMotorM2 = false,
	.sParameters.bDefaultMainMotor = M1,
	.sParameters.bMode = SINGLE_MOTOR,
	.sParameters.bCtrlType = TORQUE_CTRL,
	.sParameters.bM2TorqueInversion = false,
	.sParameters.hTorqueRampTimeUp = 200,
	.sParameters.hTorqueRampTimeDown = 50,
	.sParameters.hSpeedRampTimeUp = 200,
	.sParameters.hSpeedRampTimeDown = 50,
	.sParameters.hStartingThrottle = 1000,
	.sParameters.hStoppingThrottle = 500,
	.sParameters.hStoppingSpeed = 0,
	.sParameters.hPASMaxTorque = -7000,
	.sParameters.hPASMaxSpeed = 500,
	.sParameters.GearRatio = 0x00010001, //Ratio is unknown so 1/1 assumed
	.sParameters.hFaultManagementTimeout = 25, // Timer of 500ms for clear OC, SF and SU faults (20ms * 25)
		#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.sParameters.bUseMotorM1 = true,
	.sParameters.bUseMotorM2 = false,
	.sParameters.bDefaultMainMotor = M1,
	.sParameters.bMode = SINGLE_MOTOR,
	.sParameters.bCtrlType = TORQUE_CTRL,
	.sParameters.bM2TorqueInversion = false,
	.sParameters.hTorquePASRampTimeUp = 750,	
	.sParameters.hTorqueRampTimeUp = 200,
	.sParameters.hTorqueRampTimeDown = 50,
	.sParameters.hSpeedRampTimeUp = 200,
	.sParameters.hSpeedRampTimeDown = 50,
	.sParameters.hStartingThrottle = 1000,
	.sParameters.hStoppingThrottle = 500,
	.sParameters.hStoppingSpeed = 0,
	.sParameters.hPASMaxTorque = -10000,
	.sParameters.hPASMaxSpeed = 500,
	.sParameters.GearRatio = 0x000B0005, //Ratio is 11/5
	.sParameters.bUseWheelSpeedSensor = true,
	.sParameters.bWheelSpreedRatio = 2,
	.sParameters.hFaultManagementTimeout = 25, // Timer of 500ms for clear OC, SF and SU faults (20ms * 25)
		#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.sParameters.bUseMotorM1 = true,
	.sParameters.bUseMotorM2 = true,
	.sParameters.bDefaultMainMotor = M1,
	.sParameters.bMode = DUAL_MOTOR,
	.sParameters.bCtrlType = TORQUE_CTRL,
	.sParameters.bM2TorqueInversion = true,
	.sParameters.hTorqueRampTimeUp = 200,
	.sParameters.hTorqueRampTimeDown = 50,
	.sParameters.hSpeedRampTimeUp = 200,
	.sParameters.hSpeedRampTimeDown = 50,
	.sParameters.hStartingThrottle = 1000,
	.sParameters.hStoppingThrottle = 500,
	.sParameters.hStoppingSpeed = 0,
	.sParameters.hPASMaxTorque = -7000,
	.sParameters.hPASMaxSpeed = 500,
	.sParameters.GearRatio = 0x00010001, //Ratio is unknown so 1/1 assumed
	.sParameters.hFaultManagementTimeout = 25, // Timer of 500ms for clear OC, SF and SU faults (20ms * 25)
	#else
	.sParameters.bUseMotorM1 = true,
	.sParameters.bUseMotorM2 = false,
	.sParameters.bDefaultMainMotor = M1,
	.sParameters.bCtrlType = TORQUE_CTRL,
	.sParameters.bM2TorqueInversion = false,
	.sParameters.hTorqueRampTimeUp = 200,
	.sParameters.hTorqueRampTimeDown = 50,
	.sParameters.hSpeedRampTimeUp = 200,
	.sParameters.hSpeedRampTimeDown = 50,
	.sParameters.hStartingThrottle = 1000,
	.sParameters.hStoppingThrottle = 500,
	.sParameters.hStoppingSpeed = 0,
	.sParameters.hPASMaxTorque = -10000,
	.sParameters.hPASMaxSpeed = 500,
	.sParameters.GearRatio = 0x00010001, //Ratio is unknown so 1/1 assumed	
	.sParameters.hFaultManagementTimeout = 25, // Timer of 500ms for clear OC, SF and SU faults (20ms * 25)
	#endif
	
	.pMDI = &MDInterfaceHandle,
	.pThrottle = &ThrottleHandle,
	.pBrake = &BrakeHandle,
	.pMS = &MotorSelectorHandle,
	.pPWREN = &PowerEnableHandle,
	.pPAS = &PedalAssistHandle,
	
	.sSpeedFoldback[M1] = 
	{
		.bEnableFoldback = false,
		.hStartValue = 200,
		.hIntervalValue = 200,
		.hEndValue = 400,
		.hDefaultMaxTorque = 10000,
	},
	.sSpeedFoldback[M2] = 
	{
		.bEnableFoldback = false,
		.hStartValue = 200,
		.hIntervalValue = 200,
		.hEndValue = 400,
		.hDefaultMaxTorque = 10000,
	},
	.sHeatsinkTempFoldback[M1] = 
	{
		.bEnableFoldback = false,
		.hStartValue = 45,
		.hEndValue = 70,
		.hDefaultMaxTorque = 17000,
	},
	.sHeatsinkTempFoldback[M2] = 
	{
		.bEnableFoldback = false,
		.hStartValue = 45,
		.hEndValue = 70,
		.hDefaultMaxTorque = 17000,
	},
};

VCI_Handle_t VCInterfaceHandle = 
{
	.pStateMachine = &VCStateMachineHandle,
	.pPowertrain = &PowertrainHandle,
};

