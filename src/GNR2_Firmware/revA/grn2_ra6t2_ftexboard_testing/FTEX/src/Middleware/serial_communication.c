/**
  ******************************************************************************
  * @file    serial_communication.c
  * @brief   This file contain the Serial Communication bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "serial_communication.h"

/* Temporary buffer to save data from receive buffer for further processing */
uint8_t g_temp_buffer[DATA_LENGTH] = {RESET_VALUE};

/* Counter to update g_temp_buffer index */
volatile uint8_t g_counter_var = RESET_VALUE;

/* Flag to check whether data is received or not */
volatile uint8_t g_data_received_flag = false;

/* Flag for user callback */
volatile uint8_t g_uart_event = RESET_VALUE;
// ==================== Public function prototypes ======================== //

/**
  * @brief  Initialize  UART
  * @param  None
  * @return Upon successful open and start of timer
  */
fsp_err_t Init_uart(void)
{
    fsp_err_t  bIsError = FSP_SUCCESS;

    // Initialise the UART 
    bIsError |= R_SCI_B_UART_Open(&g_uart1_ctrl, &g_uart1_cfg);

    return bIsError;
}

/**
  * @brief  Deinitialize SCI UART module
  * @param  None
  * @return Upon successful open and start of timer
  */
fsp_err_t Deinit_uart(void)
{
    fsp_err_t  bIsError = FSP_SUCCESS;

    /* Close module */
    bIsError |= R_SCI_B_UART_Close (&g_uart1_ctrl);
    
    return bIsError;
}

/**
  * @brief  print user message to terminal
  * @param  p_msg
  * @return Upon success or Upon event failure
  */

fsp_err_t UART_SerialPrint(uint8_t *p_msg)
{
    fsp_err_t bIsError   = FSP_SUCCESS;
    uint8_t msg_len = RESET_VALUE;
    uint32_t local_timeout = (DATA_LENGTH * UINT16_MAX);
    char *p_temp_ptr = (char *)p_msg;

    /* Calculate length of message received */
    msg_len = ((uint8_t)(strlen(p_temp_ptr)));

    /* Reset callback capture variable */
    g_uart_event = RESET_VALUE;

    /* Writing to terminal */
    bIsError = R_SCI_B_UART_Write (&g_uart1_ctrl, p_msg, msg_len);


    /* Check for event transfer complete */
    while ((UART_EVENT_TX_COMPLETE != g_uart_event) && (--local_timeout))
    {
        /* Check if any error event occurred */
        if (UART_ERROR_EVENTS == g_uart_event)
        {
            return FSP_ERR_TRANSFER_ABORTED;
        }
    }
    if(RESET_VALUE == local_timeout)
    {
        bIsError = FSP_ERR_TIMEOUT;
    }
    return bIsError;
}

/**
  * @brief  Interrupt routine of UART module
  * @param  p_args: UART callback function arguments.
  * @return None
  */
void UART_IRQHandler(uart_callback_args_t * p_args)
{
    /* Logged the event in global variable */
    g_uart_event = (uint8_t)p_args->event;

    /* Reset g_temp_buffer index if it exceeds than buffer size */
    if(DATA_LENGTH == g_counter_var)
    {
        g_counter_var = RESET_VALUE;
    }

    if(UART_EVENT_RX_CHAR == p_args->event)
    {
        switch (p_args->data)
        {
            /* If Enter is pressed by user, set flag to process the data */
            case CARRIAGE_ASCII:
            {
                g_counter_var = RESET_VALUE;
                g_data_received_flag  = true;
                break;
            }
            /* Read all data provided by user until enter button is pressed */
            default:
            {
                g_temp_buffer[g_counter_var++] = (uint8_t ) p_args->data;
                break;
            }
        }
    }
}
