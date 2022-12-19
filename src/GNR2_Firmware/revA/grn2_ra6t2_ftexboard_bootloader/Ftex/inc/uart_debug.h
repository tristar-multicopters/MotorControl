/**
  ******************************************************************************
  * @file    uart_debug.h
  * @brief   This file contain the external uart debug header
  ******************************************************************************
*/

#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

// =============================== Includes ================================ //
#include "bsp_api.h"
#include "hal_data.h"
#include "stdio.h"
// ================================ Defines ============================== //
#define test_frame_length 14
#define MSG_LOG_DBG(_fmt, ...)   printf("[BOOT] " _fmt "\r\n", ##__VA_ARGS__);

// ==================== Public function prototypes ======================== //

/**
  * @brief  Initializes the UART Communication 
  * @param  None
  * @retval None
  */
void uCAL_UART_Init(void);

/**
  * @brief  Add for STDout for printf debugging
  * @param  ch byte buffer to write
  * @retval None
  */
void stdout_putchar(uint8_t ch);

#endif /* __UART_DEBUG_H */