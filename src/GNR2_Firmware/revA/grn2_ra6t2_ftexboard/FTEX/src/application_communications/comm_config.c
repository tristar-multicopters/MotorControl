/**
  * @file    comm_config.c
  * @brief   This file instantiate handles required by communication layer.
  *
*/

#include "comm_config.h"
#include "gnr_parameters.h"
#include "board_hardware.h"
#include "co_gnr2_specs.h"

// Handle of the CANOpen node
CO_NODE CONodeGNR;

// Handle of the motor 2. Motor controller 2 is outside of this ganrunner device.
SlaveMotorHandle_t SlaveM2;

UART_Handle_t UART0Handle =
{
    .UARTBaudrate = BAUD9600,
    .UARTProtocol = UART_APT,
    .pUARTInstance = &g_uart9,
};

APT_Handle_t LCD_APT_handle;
LogHighSpeed_Handle_t LogHS_handle;
