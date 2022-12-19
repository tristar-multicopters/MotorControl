/**
  ******************************************************************************
  * @file    uart_debug.c
  * @brief   This file contain the external uart debug functions
  ******************************************************************************
*/

// =============================== Includes ================================ //

#include "uart_debug.h"

// =============================== Variables ============================== //

extern volatile bool bUart_DBGFree;

// ==================== Public function prototypes ======================== //

/**
	Initializes the UART Communication 
*/
void uCAL_UART_Init(void)
{
	R_SCI_B_UART_Open(&g_uart1_ctrl, &g_uart1_cfg);
}

/**
	Add for STDout for printf debugging
*/
void stdout_putchar(uint8_t ch)
{
    uint32_t timestamp;
    // Flag free DBG printf transmit
    bUart_DBGFree = false;
    // Link the uart transmit to the stdout function
    R_SCI_B_UART_Write(&g_uart1_ctrl, &ch , 1);
    // Timeout check 
    timestamp = DWT->CYCCNT;
    while(!bUart_DBGFree && (timestamp + 10 > DWT->CYCCNT));
}
