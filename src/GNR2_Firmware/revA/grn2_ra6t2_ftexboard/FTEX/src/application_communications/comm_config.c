/**
  * @file    comm_config.c
  * @brief   This
  *
*/

#include "comm_config.h"
#include "gnr_parameters.h"
#include "board_hardware.h"

UART_Handle_t UART0Handle =
{
    .UARTBaudrate = BAUD115200,
    .UARTProtocol = UART_LOG_HS,
    .pUARTInstance = &g_uart9,
};

APT_Handle_t LCD_APT_handle;
CO_NODE CONodeGNR;
LogHighSpeed_Handle_t LogHS_handle;
