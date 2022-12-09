/**
  * @file    uart_test.h
  * @brief   This file is the device firmware test header
	*/

#ifndef __UART_TEST_H
#define __UART_TEST_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_api.h"
#include "hal_data.h"

/* Defines --------------------------------------------------------------------*/
#define test_frame_length 14

/* Function Prototypes ------------------------------------------------------------*/

/**
	Deinitializes firmware storage 
*/
void uart_init_test (void);

/**
	Deinitializes firmware storage 
*/
void uart_test (void);




#endif /* __UART_TEST_H */