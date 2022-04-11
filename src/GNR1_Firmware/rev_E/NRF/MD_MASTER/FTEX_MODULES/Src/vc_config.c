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
		
		.hOffsetThrottle = 9800,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -7,
		.bDivisorTorque = 25,
	}
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 9800,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -7,
		.bDivisorTorque = 45,
	}
		#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 9900,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -8,
		.bDivisorTorque = 48,
	}
	
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.hParam =
	{
		.hLowPassFilterBW1 = 8,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 9900,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = 8,
		.bDivisorTorque = 32,
	}
	#else
	.hParam =
	{
		.hLowPassFilterBW1 = 16,
		.hLowPassFilterBW2 = 2,
	
		.hOffsetThrottle = 9600,
		.bSlopeThrottle = 5,
		.bDivisorThrottle = 3,
		
		.hOffsetTorque = 4000,
		.bSlopeTorque = -7,
		.bDivisorTorque = 25,
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
	.hParam =
	{
		.hLowPassFilterBW1 = 16,
		.hLowPassFilterBW2 = 2,
		.hOffset = 12000,
		.hMax = UINT16_MAX,
		.m = -16,
		.F = 35,
	}
};
/**@brief Speed pulse Pin initializing Parameters.
 */
SPR_Handle_t SpeedPulse =
{
	.bCaptureChannel = 6,
	.bRestartChannel = 5,
	
	.WCaptureChannel = 4,
	.WRestartChannel = 3,
	
	.bTimer_Prescaler = NRF_TIMER_FREQ_1MHz,
	.bTimer_Width = NRF_TIMER_BIT_WIDTH_16,

	.pTimerInstance = PAS_TIMER_INSTANCE_ADDR,
	.wTimerInstance = WH_TIMER_INSTANCE_ADDR,
	
	.pSinSpeed_Pulse_pin = PAS_SIN_GPIO_PIN,
	.pCosSpeed_Pulse_pin = PAS_COS_GPIO_PIN,
	.pWheelSpeed_Pulse_pin = WH_PUL_GPIO_PIN,
	.sParam =
	{
		.sLowPassFilterBW1 = 16,
		.sMax = UINT16_MAX,
		.sWheel_Lap_Count = 0,
		.sFirst_Wheel_Lap = false,
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

/**@brief Pedal assist initializing Parameters.
 */
PAS_Handle_t PedalAssistHandle = {
	.pTorque = &TorqueSensor,
	.pSpulse = &SpeedPulse,
	.pRampCoeff = 50,
};

PWREN_Handle_t PowerEnableHandle = {
	.wPinNumber = PWREN_GPIO_PIN,
	.bIsInvertedLogic = false,
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bUsePowerLock = true,
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bUsePowerLock = false,
	#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bUsePowerLock = false,
	#elif VEHICLE_SELECTION == VEHICLE_GEEBEECARGO
	.bUsePowerLock = false,
	#else
	.bUsePowerLock = true;
	#endif
};

DRVT_Handle_t DrivetrainHandle = 
{	
	#if VEHICLE_SELECTION == VEHICLE_ECELL
	.bUseMotorM1 = true,
	.bUseMotorM2 = true,
	.bDefaultMainMotor = M1,
	.bMode = DUAL_MOTOR,
	.bCtrlType = TORQUE_CTRL,
	.bM2TorqueInversion = false,
	.hTorqueRampTimeUp = 200,
	.hTorqueRampTimeDown = 50,
	.hSpeedRampTimeUp = 200,
	.hSpeedRampTimeDown = 50,
	.hStartingThrottle = 1000,
	.hStoppingThrottle = 500,
	.hStoppingSpeed = 0,
	.hMaxTorque = -10000,
	.hMaxLevel	=	5,
	#elif VEHICLE_SELECTION == VEHICLE_EBGO
	.bUseMotorM1 = true,
	.bUseMotorM2 = false,
	.bDefaultMainMotor = M1,
	.bMode = SINGLE_MOTOR,
	.bCtrlType = TORQUE_CTRL,
	.bM2TorqueInversion = false,
	.hTorqueRampTimeUp = 200,
	.hTorqueRampTimeDown = 50,
	.hSpeedRampTimeUp = 200,
	.hSpeedRampTimeDown = 50,
	.hStartingThrottle = 1000,
	.hStoppingThrottle = 500,
	.hStoppingSpeed = 0,
	.hMaxTorque = -7000,
	.hMaxLevel	=	5,
		#elif VEHICLE_SELECTION == VEHICLE_GRIZZLY
	.bUseMotorM1 = true,
	.bUseMotorM2 = false,
	.bDefaultMainMotor = M1,
	.bMode = SINGLE_MOTOR,
	.bCtrlType = TORQUE_CTRL,
	.bM2TorqueInversion = false,
	.hTorqueRampTimeUp = 200,
	.hTorqueRampTimeDown = 50,
	.hSpeedRampTimeUp = 200,
	.hSpeedRampTimeDown = 50,
	.hStartingThrottle = 1000,
	.hStoppingThrottle = 500,
	.hStoppingSpeed = 0,
	.hMaxTorque = -10000,
	.hMaxLevel	=	5,
	.pAvTorque =0,			
	.pLowPassFilterBW1 = 50,	
	.pLowPassFilterBW2 = 1,		
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
	.sParameters.hFaultManagementTimeout = 25 // Timer of 500ms for clear OC, SF and SU faults (20ms * 25)
	.sParameters.hMaxTorque = -10000,
	.sParameters.hMaxLevel	=	5,
	#endif
	
	.pMDI = &MDInterfaceHandle,
	.pThrottle = &ThrottleHandle,
	.pPAS = &PedalAssistHandle,
	.pBrake = &BrakeHandle,
	.pMS = &MotorSelectorHandle,
	.pPWREN = &PowerEnableHandle,
	
};

VCI_Handle_t VCInterfaceHandle = 
{
	.pStateMachine = &VCStateMachineHandle,
	.pDrivetrain = &DrivetrainHandle,
};

eUART_protocol_t EUART_handle_t = EUART_DISABLE;
//LCD_handle_t BafangScreenHandle = 
//{
//	.pVCInterface = &VCInterfaceHandle,
//};
	
