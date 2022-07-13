/**
*  uCAL_UART.c
*  uController Abstraction Layer for UART
*/ 
 
#include "uCAL_UART.h"
#include "ASSERT_FTEX.h"

#define UART_BAUDRATE_ERROR_PERCENT_5 5000
 

// ==================== Public Functions ======================== //
 
/**
  Function used to initialise the UART module
*/
void uCAL_UART_Init(UART_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL); 

    uCAL_UART_SetBaudRate(pHandle);
}

/**
  Function used to set the UART baud rate using the renesas API
*/
void uCAL_UART_SetBaudRate(UART_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL); 
    sci_b_baud_setting_t baud_setting;
    uint32_t             baud_rate                 = pHandle->UARTBaudrate;
    bool                 enable_bitrate_modulation = false;
    uint32_t             error_rate_x_1000         = UART_BAUDRATE_ERROR_PERCENT_5;
    
    //Calculating the baud rate settings to be place in the registers accoridng to the required speed
    fsp_err_t err = R_SCI_B_UART_BaudCalculate(baud_rate, enable_bitrate_modulation, error_rate_x_1000, &baud_setting);
    ASSERT(FSP_SUCCESS == err);
    
    //Setting the registers accoridng to the previously calculated values
    err = R_SCI_B_UART_BaudSet(pHandle->UARTInsance->p_ctrl, (void *) &baud_setting);
    ASSERT(FSP_SUCCESS == err);

}

/**
  Function used to transmit data via UART from a specific buffer
*/
void uCAL_UART_Transmit(UART_Handle_t *pHandle, uint8_t *Buffer, uint32_t BufferSize)
{
    ASSERT(pHandle != NULL); 
    //Send the values via UART using Renesas API
    fsp_err_t err = R_SCI_B_UART_Write(pHandle->UARTInsance->p_ctrl, Buffer, BufferSize);
    ASSERT(FSP_SUCCESS == err);    
}

/**
  Function used to receive data via UART in a specific buffer
*/
void uCAL_UART_Receive(UART_Handle_t *pHandle, uint8_t *Buffer, uint32_t BufferSize)
{
    ASSERT(pHandle != NULL); 
    //Receive the values via UART using Renesas API
    fsp_err_t err = R_SCI_B_UART_Read(pHandle->UARTInsance->p_ctrl, Buffer, BufferSize);
    ASSERT(FSP_SUCCESS == err);
}
