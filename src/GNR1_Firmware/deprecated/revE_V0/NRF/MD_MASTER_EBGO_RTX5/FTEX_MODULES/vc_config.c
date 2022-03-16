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
	.DeratingHandler = 
	{
		.hTempThreshold = 50,
		.hSlope = 500,
	},
};

//*******************************************************************************
//	Motor 2
//*******************************************************************************

MD_Handle_t MotorDrive2 = 
{
	.DeratingHandler = 
	{
		.hTempThreshold = 50,
		.hSlope = 500,
	},
};

//*******************************************************************************
//	Vehicle
//*******************************************************************************

SPI_Handle_t SPI0Manager = 
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

MCP25625_Handle_t CANController =
{
	.pSPI = &SPI0Manager,
	.device = {
		.ss_pin = MCP_SS_PIN,
	},
	.baudrate = CAN_250KBS,
	.int_pin = MCP_INT_PIN,
	.state = MCP_IDLE
};

Brake_Handle_t BrakeSensor = 
{ 
	.wPinNumber = BRAKE_GPIO_PIN,
	.bIsInvertedLogic = true,
};

RCM_Handle_t RegularConvertionManager =
{
	.pTimerInstance = RCM_TIMER_INSTANCE_ADDR,
	.samplingTime_ms = 10,
};

Throttle_Handle_t ThrottleSensor =
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
		.resp_type = LINEAR,
		.hLowPassFilterBW  = 4,
		.hOffset = 12000,
		.hMax = UINT16_MAX,
		.m = 17, //17,
		.F = 36,
	}
};

TD_Handle_t TorqueDistributor = 
{
	#if USE_MOTOR1
	.bMainMotorSelection = M1,
	#else
	.bMainMotorSelection = M2,
	#endif
	
	#if USE_MOTOR1 && USE_MOTOR2
	.bMode = DUAL_MOTOR,
	#else
	.bMode = SINGLE_MOTOR,
	#endif
	
	.wM1SelectPinNumber = M1SELECT_GPIO_PIN,
	.wM2SelectPinNumber = M2SELECT_GPIO_PIN,
};

VehiculeParameters_t VehiculeParam =
{ 0
};	

VCSTM_Handle_t VCStateMachine = 
{	0
};

MD_Comm_Handle_t MDComm = 
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

HOST_Comm_Handle_t HOSTCommHandler = 
{ 
	.UARTconfig = 
	{
		.ufcp_handle = 
		{
			.p_uart_inst = UART0_INSTANCE_ADDR,
		},
		.tx_pin = UART0_TX_PIN,
		.rx_pin = UART0_RX_PIN,
	}
};

PAS_Handle_t PedalAssistHandler = {

 .hIqmax = 12000,
 .hNumberOfLevel = PAS_LEVEL_3,
 .count = 0, 
 .pas_config = {
									.pTimerInstance = PAS_TIMER_INSTANCE_ADDR,
									.pas_timer_timeout_ms = 500,
									.pas_speed_encoder_pin = PAS_SIN_GPIO_PIN,
									.pas_type = PAS_TORQUE,
								},
};

VC_Handle_t VController = 
{
	.pBrake = &BrakeSensor,
	.pThrottle = &ThrottleSensor,
	.pTorqueDistributor = &TorqueDistributor,
	.pVehiculeParam = &VehiculeParam,
	.pMDComm = &MDComm,
	.pSTM = &VCStateMachine,
	.pPedalAssist = &PedalAssistHandler,
};

