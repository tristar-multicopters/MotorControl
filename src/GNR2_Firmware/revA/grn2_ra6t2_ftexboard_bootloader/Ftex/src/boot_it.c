/**
  ******************************************************************************
  * @file    boot_it.c
  * @brief   This file present the Interrupt Service Routines.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "hal_data.h"

// ========================== Public Variables ============================ //
volatile bool bTransfer_Complete = false;
volatile bool bUart_DBGFree = true;
// ==================== Public function prototypes ======================== //

/**
  * @brief  Interrupt routine for SPI communication
  * @param  p_args: callback function arguments
*/
void SPI_IRQHandler(spi_callback_args_t * p_args)
{
    if (p_args->event == SPI_EVENT_TRANSFER_COMPLETE)
    {
        bTransfer_Complete = true;
    }
}

/**
  * @brief  Interrupt routine for UART communication
  * @param  p_args: callback function arguments
*/
void UART_IRQHandler (uart_callback_args_t * p_args)
{
	/* Handle the UART event */
    if(p_args->event == UART_EVENT_TX_DATA_EMPTY)
    {
        bUart_DBGFree = true;
        R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MILLISECONDS);	
    }
}