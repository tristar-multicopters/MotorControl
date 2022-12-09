/**
  * @file    test.c
  * @brief   This file contain the firmware test uart file.
  */
	
/* Includes ------------------------------------------------------------------*/
#include "uart_test.h"


/* Private Variables ------------------------------------------------------------*/
uint8_t test_frame [test_frame_length] = "DENI Bootloadr";

/* Functions ---------------------------------------------------------------- */
/**
	Deinitializes firmware storage 
*/
void uart_init_test (void)
{
	R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
}


/**
	Deinitializes firmware storage 
*/
void uart_test (void)
{
	
	while (true)
	{
	
		for (int i=0; i<test_frame_length; i++)
		{
			R_SCI_B_UART_Write(&g_uart9_ctrl, (uint8_t*) &test_frame[i], test_frame_length);
			R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
		}
	}
}

void UART_IRQHandler(uart_callback_args_t * p_args)
{

}

