/**
  ******************************************************************************
  * @file    board_hardware.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module defines the hardware (pins, peripherals, ...) used by the board. 
	* 				 It creates as global variables the nrf_drv peripheral instances.
  *
	******************************************************************************
	*/

#include "board_hardware.h"


nrf_drv_uart_t uart1_inst = NRF_DRV_UART_INSTANCE(1);

nrf_drv_timer_t timer1_inst = NRF_DRV_TIMER_INSTANCE(1);

nrf_drv_spi_t spi0_inst = NRF_DRV_SPI_INSTANCE(0);

