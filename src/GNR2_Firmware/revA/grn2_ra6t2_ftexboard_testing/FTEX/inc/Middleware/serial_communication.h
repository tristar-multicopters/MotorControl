/**
  ******************************************************************************
  * @file    Serila_Communication.h
  * @brief   This file contain the serial communcation UART driver header
  ******************************************************************************
*/
#ifndef __SERIAL_COMMUNICATION_H
#define __SERIAL_COMMUNICATION_H

// =============================== Includes ================================ //
#include "hal_data.h"
#include "stdbool.h"


// ================================ Defines =============================== //

#define CARRIAGE_ASCII          (13u)     /* Carriage return */
#define ZERO_ASCII              (48u)     /* ASCII value of zero */
#define NINE_ASCII              (57u)     /* ASCII value for nine */


#define RESET_VALUE             (0x00)
#define DATA_LENGTH             (20u)      /* Expected Input Data length */
#define UART_ERROR_EVENTS       (UART_EVENT_BREAK_DETECT | UART_EVENT_ERR_OVERFLOW | UART_EVENT_ERR_FRAMING | \
                                 UART_EVENT_ERR_PARITY)    /* UART Error event bits mapped in registers */
// ==================== Public function prototypes ======================== //

/**
  * @brief  Initialize SCI UART module
  * @param  None
  * @return Upon successful open and start of timer
  */
fsp_err_t Init_uart(void);

/**
  * @brief  Deinitialize SCI UART module
  * @param  None
  * @return Upon successful open and start of timer
  */
fsp_err_t Deinit_uart(void);

/**
  * @brief  print user message to terminal
  * @param  p_msg
  * @return Upon success or Upon event failure
  */

fsp_err_t UART_SerialPrint(uint8_t *p_msg);

/**
  * @brief  Interrupt routine of UART module
  * @param  p_args: UART callback function arguments.
  * @return None
  */
void UART_IRQHandler(uart_callback_args_t * p_args);


#endif /* __SERIAL_COMMUNICATION_H */