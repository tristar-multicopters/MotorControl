/**
  * @file    test.c
  * @brief   This file contain the firmware test uart file.
  *
  */
#include "test.h"

bool uart_dbg_free = true;

uint8_t test_frame [test_frame_length] = "I am available";

/**
	Initializes the UART Communication 
*/
void Uart_Init (void)
{
	R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
}

/**
	UART Test Function  
*/
void Uart_Test (void)
{
	while (true)
	{
		for (int i=0; i<test_frame_length; i++)
		{
			R_SCI_B_UART_Write(&g_uart9_ctrl, (uint8_t*) &test_frame[i], test_frame_length);
			R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
		}
	}
}

/**
	UART Interrupt Call
*/
void UART_IRQHandler (uart_callback_args_t * p_args)
{
	/* Handle the UART event */
	if(p_args->event == UART_EVENT_TX_DATA_EMPTY)
	{
		uart_dbg_free = true;
		R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MILLISECONDS);	
	}
}

/**
	Add for STDout for printf debugging
*/
void stdout_putchar(uint8_t ch)
{
  uint32_t timestamp;
	uart_dbg_free = false;
	R_SCI_B_UART_Write(&g_uart9_ctrl, &ch , 1);
	timestamp = DWT->CYCCNT;
	while(!uart_dbg_free && (timestamp + 10 > DWT->CYCCNT));
}
