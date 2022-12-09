/**
  * @file    boot_it.c
  * @brief   Interrupt Service Routines.
*/

/* Includes ------------------------------------------------------------------*/
#include "ASSERT_FTEX.h"
#include "hal_data.h"

volatile bool g_transfer_complete = false;


/**
  * @brief  Interrupt routine of SPI communication
  * @param  p_args: callback function arguments
*/
void spi_callback(spi_callback_args_t * p_args)
{
	if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
	{
			g_transfer_complete = true;
	}
}