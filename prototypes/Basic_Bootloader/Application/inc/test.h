/**
  * @file    test.h
  * @brief   This file is the device firmware test header
	*/

#ifndef __TEST_H
#define __TEST_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_api.h"
#include "hal_data.h"
#include "stdio.h"
/* Defines --------------------------------------------------------------------*/
#define test_frame_length 14

/* Function Prototypes ------------------------------------------------------------*/
#define MSG_LOG_DBG(_fmt, ...)   printf("[BOOT] " _fmt "\r\n", ##__VA_ARGS__);

/**
	Deinitializes firmware storage 
*/
void Uart_Init (void);

/**
	Deinitializes firmware storage 
*/
void Uart_Test (void);

/**
	UART Interrupt Call
*/
void UART_IRQHandler (uart_callback_args_t * p_args);

/**
	Add for STDout for printf debugging
*/
void stdout_putchar(uint8_t ch);

#endif /* __TEST_H */