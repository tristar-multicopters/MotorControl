/**
  * @file    comm_config.c
  * @brief   This file instantiate handles required by communication layer.
  *
*/

#include "comm_config.h"
#include "gnr_parameters.h"
#include "co_gnr2_specs.h"


// Handle of the CANOpen node
CO_NODE CONodeGNR;

// Handle of the motor 2. Motor controller 2 is outside of this ganrunner device.
SlaveMotorHandle_t SlaveM2;

UART_Handle_t UART0Handle =
{
    //.UARTBaudrate = BAUD115200,   //for HS Log
    .UARTBaudrate = BAUD9600,       // for APT
    //.UARTProtocol = UART_LOG_HS,    // for HS Log
    .UARTProtocol = UART_APT,     // for APT  
    .pUARTInstance = &g_uart9,
};

APT_Handle_t LCD_APT_handle;
LogHighSpeed_Handle_t LogHS_handle;

//Handle to control the data flash initialisation 
DataFlash_Handle_t DataFlashHandle =
{
	
	.pFlashInstance = &g_flash0,
	.dataFlashOpenFlag = false,

};

