/**
  ******************************************************************************
  * @file    vc_config.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module declares global structures used by other vehicule control modules
  *
	******************************************************************************
*/

#include "vc_config.h"


//*******************************************************************************
//	Motor 1
//*******************************************************************************

MD_Handle_t MotorDrive1 = 
{
	0
};

//*******************************************************************************
//	Motor 2
//*******************************************************************************

MD_Handle_t MotorDrive2 = 
{
	0
};

//*******************************************************************************
//	Vehicle
//*******************************************************************************

SPI_Handle_t SPI0Handle = 
{
	.p_spi_inst = SPI0_INSTANCE_ADDR,
	.spi_config = {
		.ss_pin = NRF_DRV_SPI_PIN_NOT_USED,
		.miso_pin = SPI0_MISO_PIN,
		.mosi_pin = SPI0_MOSI_PIN,
		.sck_pin  = SPI0_SCK_PIN,
		.mode = NRF_DRV_SPI_MODE_0,
		.frequency = NRF_DRV_SPI_FREQ_2M,
		.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
		.orc = 0xFF,
	},
	.nb_of_dev = 0,
};

MCP25625_Handle_t CANControllerHandle =
{
	.pSPI = &SPI0Handle,
	.device = {
		.ss_pin = MCP_SS_PIN,
	},
	.baudrate = CAN_250KBS,
	.int_pin = MCP_INT_PIN,
	.stby_pin = CAN_STBY_PIN,
	.state = MCP_IDLE
};

RCM_Handle_t RegularConvertionManager =
{
	.pTimerInstance = RCM_TIMER_INSTANCE_ADDR,
	.samplingTime_ms = 10,
};

BRK_Handle_t BrakeHandle = 
{ 
	.wPinNumber = BRAKE_GPIO_PIN,
	.bIsInvertedLogic = true,
};

THRO_Handle_t ThrottleHandle =
{
	.pRegularConversionManager = &RegularConvertionManager,
	.hChannelConfig =
	{
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain = NRF_SAADC_GAIN1_6,
		.reference = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time = NRF_SAADC_ACQTIME_10US,
		.mode = NRF_SAADC_MODE_SINGLE_ENDED,
		.burst = NRF_SAADC_BURST_DISABLED,
		.pin_p = THROTTLE_ANALOG_PIN,
		.pin_n = NRF_SAADC_INPUT_DISABLED,
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
		.bSlopeTorque = -14,///15
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


/**@brief Torque Pin initializing Parameters.
 */
TS_Handle_t TorqueSensor =
{
	.pRegularConversionManager = &RegularConvertionManager,
	.hChannelConfig =
	{
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain = NRF_SAADC_GAIN1_6,
		.reference = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time = NRF_SAADC_ACQTIME_10US,
		.mode = NRF_SAADC_MODE_SINGLE_ENDED,
		.burst = NRF_SAADC_BURST_DISABLED,
		.pin_p = PAS_TORQUE_PIN,
		.pin_n = NRF_SAADC_INPUT_DISABLED,
	},
#if VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.hParam =
	{	
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 14,
		
		.hOffsetTS = 12500,
		.bSlopeTS = 5,			
		.bDivisorTS = 4,
		
		.hOffsetMT = 20000, /* Offset to launch the torque sensing */
		.bSlopeMT = -7,
		.bDivisorMT = 2,
			
		.hMax = UINT16_MAX,
	}
#endif
};
/**@brief Speed pulse Pin initializing Parameters.
 */
SPR_Handle_t PedalSpeedPulse =
{
	.bCaptureChannel = 7,
	.bRestartChannel = 6,
	
	.bTimer_Prescaler = NRF_TIMER_FREQ_1MHz,
	.bTimer_Width = NRF_TIMER_BIT_WIDTH_32,

	.pTimerInstance = PAS_TIMER_INSTANCE_ADDR,
	
	.pSinSpeed_Pulse_pin = PAS_SIN_GPIO_PIN,
	.pCosSpeed_Pulse_pin = PAS_COS_GPIO_PIN,
	.sParam =
	{
		.sLowPassFilterBW1 = 4,
		.sLowPassFilterBW2 = 1,
		.sFirst_Wheel_Lap = false,
	}
	
};


WPR_Handle_t WheelSpeedPulse =
{

	.WCaptureChannel = 0,
	.WRestartChannel = 1,
	
	.wTimer_Prescaler = NRF_TIMER_FREQ_1MHz,
	.wTimer_Width = NRF_TIMER_BIT_WIDTH_32,

	.wTimerInstance = WH_TIMER_INSTANCE_ADDR,
	
	.pWheelSpeed_Pulse_pin = WH_PUL_GPIO_PIN,
	.wParam =
	{
		.hWLowPassFilterBW1 = 4,
		.hWLowPassFilterBW2 = 1,
		.wWheel_Lap_Count = 0,
	}
	
};



MS_Handle_t MotorSelectorHandle = 
{
	.wM1SelectPinNumber = M1SELECT_GPIO_PIN,
	.wM2SelectPinNumber = M2SELECT_GPIO_PIN,
	
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

VCSTM_Handle_t VCStateMachineHandle = 
{	
	.bVState = V_IDLE,
	.hVFaultNow = 0,
	.hVFaultOccurred = 0
};

MD_Comm_Handle_t MDCommunicationHandle = 
{ 
	.pMD = {&MotorDrive1, &MotorDrive2},
	
	.UARTconfig = 
	{
		.ufcp_handle = 
		{
			.p_uart_inst = MD_UART_INSTANCE_ADDR,
		},
		.tx_pin = MD_UART_TX_PIN,
		.rx_pin = MD_UART_RX_PIN,
	}
};

MDI_Handle_t MDInterfaceHandle = 
{ 
	.pMDC = &MDCommunicationHandle,
};

/**@brief Wheel Speed Sensor initializing Parameters.
 */
WSS_Handle_t WheelSpeedSensorHandle = {

	.wSpulse = &WheelSpeedPulse,
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bWSPulseNb = 1,
	.bWSSslowDetect = false,
	.bSlowDetectCount = 0,		
	.bSlowDetectCountValue = 1, 

	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bWSPulseNb = 1,
	.bWSSslowDetect = true,	
	.bSlowDetectCount = 0,		
	.bSlowDetectCountValue = 5,
	
	#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bWSPulseNb = 3,
	.bWSSslowDetect = false,	
	.bSlowDetectCount = 0,		
	.bSlowDetectCountValue = 1, 
	
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.bWSPulseNb = 0,
	#else
	.bWSPulseNb = 0,
	
	#endif
	
	
	
};
/**@brief Pedal assist initializing Parameters.
 */
PAS_Handle_t PedalAssistHandle = {
	.pTorque = &TorqueSensor,
	.pSpulse = &PedalSpeedPulse,

	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bMaxLevel	=	5,
	.bPulseNb	= 0,	
	.bTorqueSensorUse = false,
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bMaxLevel	=	5,
	.bPulseNb	= 30,	// NUMBER_OF_PINS of pulse / rotation
	.bTorqueSensorUse = false,
	#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bMaxLevel	=	5,
	.bMaxTorque = -17000,
	.bPulseNb	= 0,	
	.bTorqueSensorUse = true,
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.bMaxLevel	=	5,
	#else
	.bMaxLevel	=	5,
	#endif
};

PWREN_Handle_t PowerEnableHandle = {
	.wPinNumber = PWREN_GPIO_PIN,
	.bIsInvertedLogic = false,
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bUsePowerLock = true,
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bUsePowerLock = true,
	#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bUsePowerLock = true,                   
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.bUsePowerLock = false,
	#else
	.bUsePowerLock = true;
	#endif
};

DRVT_Handle_t DrivetrainHandle = 
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
	.sParameters.hTorquePASRampTimeUp = 1250,
	.sParameters.hTorqueRampTimeUp = 200,
	.sParameters.hTorqueRampTimeDown = 50,
	.sParameters.hSpeedRampTimeUp = 200,
	.sParameters.hSpeedRampTimeDown = 50,
	.sParameters.hStartingThrottle = 1000,
	.sParameters.hStoppingThrottle = 500,
	.sParameters.hStoppingSpeed = 0,
	.sParameters.hPASMaxTorque = -7000,
	.sParameters.hPASMaxSpeed = 400,
	.sParameters.bUseWheelSpeedSensor = true,
	.sParameters.bWheelSpreedRatio = 2,
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
	.pPAS = &PedalAssistHandle,
	.pWSS = &WheelSpeedSensorHandle,
	.pBrake = &BrakeHandle,
	.pMS = &MotorSelectorHandle,
	.pPWREN = &PowerEnableHandle,
	
	.sSpeedFoldback[M1] = 
	{
		.bEnableFoldback = true,
		.hStartValue = 200,
		.hIntervalValue = 200,
		.hEndValue = 400,
		.hDefaultMaxTorque = 10000,
	},
	.sSpeedFoldback[M2] = 
	{
		.bEnableFoldback = true,
		.hStartValue = 200,
		.hIntervalValue = 200,
		.hEndValue = 400,
		.hDefaultMaxTorque = 10000,
	},
	.sHeatsinkTempFoldback[M1] = 
	{
		.bEnableFoldback = true,
		.hStartValue = 45,
		.hEndValue = 70,
		.hDefaultMaxTorque = 17000,
	},
	.sHeatsinkTempFoldback[M2] = 
	{
		.bEnableFoldback = true,
		.hStartValue = 45,
		.hEndValue = 70,
		.hDefaultMaxTorque = 17000,
	},
};

VCI_Handle_t VCInterfaceHandle = 
{
	.pStateMachine = &VCStateMachineHandle,
	.pDrivetrain = &DrivetrainHandle,
};

eUART_protocol_t EUART_handle_t = EUART_APT; // Has to been initialise by Evionics first

	
