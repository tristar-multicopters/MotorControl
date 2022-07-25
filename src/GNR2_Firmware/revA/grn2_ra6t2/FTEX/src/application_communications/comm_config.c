/**
  * @file    comm_config.c
  * @brief   This
  *
*/

#include "comm_config.h"
#include "gnr_parameters.h"
#include "board_hardware.h"
#include "comm_parameters.h"
#include "lcd_apt_comm.h"

UART_Handle_t UART0_handle = 
{
      .UARTBaudrate = BAUD9600,
      .UARTProtocol = UART_APT,
      .pUART_Parameters = &UART0_Parameters    
};     

APT_Handle_t LCD_APT_handle;

// CAN handler declaration
CAN_Handler_t CAN_handle = {0};