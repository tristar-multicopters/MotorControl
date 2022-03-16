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


//*******************************************************************************
//	Motor 2
//*******************************************************************************


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

MS_Handle_t MotorSelectorHandle = 
{
	.wM1SelectPinNumber = M1SELECT_GPIO_PIN,
	.wM2SelectPinNumber = M2SELECT_GPIO_PIN,
	.bIsInvertedLogic = false,
};

VCSTM_Handle_t VCStateMachineHandle = 
{	0
};

MD_Comm_Handle_t MDCommunicationHandle = 
{ 
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

PAS_Handle_t PedalAssistHandle = {
	0
};

DRVT_Handle_t DrivetrainHandle = 
{	
	.bUseMotorM1 = true,
	.bUseMotorM2 = false,
	.bDefaultMainMotor = M1,
	.bCtrlType = TORQUE_CTRL,
	.hTorqueRampTime = 200,
	.hSpeedRampTime = 200,
	
	.pMDI = &MDInterfaceHandle,
	.pThrottle = &ThrottleHandle,
	.pPAS = &PedalAssistHandle,
	.pBrake = &BrakeHandle,
	.pMS = &MotorSelectorHandle,
};

VCI_Handle_t VCInterfaceHandle = 
{
	.pStateMachine = &VCStateMachineHandle,
	.pDrivetrain = &DrivetrainHandle,
};

LCD_handle_t BafangScreenHandle = 
{
	.pVCInterface = &VCInterfaceHandle,
};
	
