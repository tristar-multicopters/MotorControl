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

nrf_drv_uart_t uart0_inst = NRF_DRV_UART_INSTANCE(0);

nrf_drv_uart_t uart1_inst = NRF_DRV_UART_INSTANCE(1);

nrf_drv_timer_t timer1_inst = NRF_DRV_TIMER_INSTANCE(1);

nrf_drv_timer_t timer2_inst = NRF_DRV_TIMER_INSTANCE(2);

nrf_drv_timer_t timer3_inst = NRF_DRV_TIMER_INSTANCE(3);

nrf_drv_spi_t spi0_inst = NRF_DRV_SPI_INSTANCE(0);

// Function used to get the chip ID
// ID_half decides which half of the ID is return
// 0 for lower half and 1 for upper half
uint32_t GetChipID(uint8_t ID_Half)
{
   uint32_t value;

   if(ID_Half > 1)
      ID_Half = 1;	 
	
   value = NRF_FICR->DEVICEID[ID_Half];

   return(value);

}



