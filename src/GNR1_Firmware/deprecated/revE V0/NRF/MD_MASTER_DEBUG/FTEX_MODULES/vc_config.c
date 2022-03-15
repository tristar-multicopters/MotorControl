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
{ 0
};

//*******************************************************************************
//	Motor 2
//*******************************************************************************

MD_Handle_t MotorDrive2 = 
{ 0
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
		.frequency = NRF_DRV_SPI_FREQ_250K,
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
	//.res120_pin = MCP_SLCT_120R
};

CAN_Handle_t CANbusManager = 
{
	.pMCP = &CANController,
};


Brake_Handle_t BrakeSensor = 
{ 
	.wPinNumber = BRAKE_GPIO_PIN,
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
		.m = 9, //15,
		.F = 32, //28,
	}
};

TD_Handle_t TorqueDistributor = 
{
	#ifdef USE_TWO_MOTORS
	.bMode = DUAL_MOTOR,
	#else
	.bMode = SINGLE_MOTOR,
	#endif
	
	.hDeadzoneValue = 2000,
	.hDeadzoneHysteresis = 1000,
};

VehiculeParameters_t VehiculeParam =
{ 0
};	

VCSTM_Handle_t VCStateMachine = 
{	0
};

MD_Comm_Handle_t MDComm = 
{ 
	.UARTconfig = 
	{
		.p_uart_inst = MD_UART_INSTANCE_ADDR,
		.tx_pin = MD_UART_TX_PIN,
		.rx_pin = MD_UART_RX_PIN,
	}
};

VC_Handle_t VControl = 
{
	.pCANbusManager = &CANbusManager,
	.pBrake = &BrakeSensor,
	.pThrottle = &ThrottleSensor,
	.pTorqueDistributor = &TorqueDistributor,
	.pVehiculeParam = &VehiculeParam,
	.pMDComm = &MDComm,
	.pSTM = &VCStateMachine,
	.pRegularConvertionManager = &RegularConvertionManager,
};

