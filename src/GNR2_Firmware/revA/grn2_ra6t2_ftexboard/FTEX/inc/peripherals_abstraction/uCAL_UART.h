/**
* @file   uCAL_UART.h
* @author Andy B
* @brief  uController Abstraction Layer for UART
*
* This module is the interface that is used by the entire firmware
* to interact with the UART. It is the bridge between the Renesas 
* Serial Communication interface in UART mode and the FTEX firmware. 
*
*/

#ifndef UCAL_UART
#define UCAL_UART

#include "stdlib.h"
#include "stdint.h"
#include "r_sci_b_uart.h"

#define UART_FLAG  0x10 //TODO move to the proper spot

// ======================== Pin configuration enums ======================== // 

typedef enum  // Used to select the UART baudrate
{
    BAUD1200   =   1200,
    BAUD9600   =   9600,
    BAUD115200 = 115200,
} uCAL_BaudRate_t;

typedef enum
{
    UART_DISABLE,
    UART_APT,
    UART_KD718,
    UART_CLOUD_5S,    
    UART_LOG_HS 

} uCAL_Protocol_t;

typedef struct
{
    const uart_instance_t * pUARTInstance;
    uCAL_BaudRate_t    UARTBaudrate;
    uCAL_Protocol_t    UARTProtocol;
    uint8_t            OpenRecpBuffer[30];
    void* Super;                    /* Pointer to the higher handle that is using the uart port
                                       MUST be initialized before using. */
    void (*pRxCallback)(void *);
    void (*pTxCallback)(void *);
}
UART_Handle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief Function used to initialise the UART using the renesas API
    
  @param Receives UART baud rate and handle
  @return void
*/
void uCAL_UART_Init(UART_Handle_t *pHandle);

/**
  @brief Function used to set the UART baud rate using the renesas API
    
  @param Receives UART baud rate
  @return void
*/
void uCAL_UART_SetBaudRate(UART_Handle_t *pHandle);

/**
  @brief Function used to transmit data via UART from a specific buffer
    
  @param Buffer pointer and size
  @return void
*/
void uCAL_UART_Transmit(UART_Handle_t *pHandle, uint8_t *Buffer, uint32_t BufferSize);

/**
  @brief Function used to receive data via UART in a specific buffer
    
  @param Buffer pointer and size
  @return void
*/
void uCAL_UART_Receive(UART_Handle_t *pHandle, uint8_t *Buffer, uint32_t BufferSize);


void uCAL_UART_IRQHandler(UART_Handle_t *pHandle,uart_callback_args_t * p_args);

#endif
