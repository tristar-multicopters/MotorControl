/**
  * @file    comm_parameters.c
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "comm_parameters.h"
#include "gnr_main.h"

UART_Parameters_t UART0_Parameters = 
{
  .UARTInstance = &g_uart9  
};