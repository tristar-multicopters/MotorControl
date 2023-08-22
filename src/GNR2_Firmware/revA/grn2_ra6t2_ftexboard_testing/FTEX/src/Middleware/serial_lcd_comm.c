/**
  ******************************************************************************
  * @file    serial_lcd_comm.c
  * @brief   This file contain the Serial Communication bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "serial_lcd_comm.h"
#include "serial_communication.h"

/* Temporary buffer to save data from receive buffer for further processing */
uint8_t bRecFrame[DATA_LENGTH] = {RESET_VALUE};

/* Counter to update bRecivedFrame index */
volatile uint8_t bCounter = RESET_VALUE;

/* Flag to check whether data is received or not */
volatile uint8_t bData_ReceivedFlag = false;

/* Flag for user callback */
volatile uint8_t bUartEvent = RESET_VALUE;

// ==================== Public function prototypes ======================== //
/**
  * @brief  Initialize  UART
  * @param  None
  * @return Upon successful open and start of timer
  */
fsp_err_t Init_uart_lcd(void)
{
    fsp_err_t  bIsError = FSP_SUCCESS;

    // Initialise the UART 
    bIsError |= R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);

    return bIsError;
}

/**
  * @brief  Deinitialize SCI UART module
  * @param  None
  * @return Upon successful open and start of timer
  */
fsp_err_t Deinit_uart_lcd(void)
{
    fsp_err_t  bIsError = FSP_SUCCESS;

    /* Close module */
    bIsError |= R_SCI_B_UART_Close (&g_uart9_ctrl);
    
    return bIsError;
}

/**
  * @brief  print user message to terminal
  * @param  p_msg
  * @return Upon success or Upon event failure
  */

fsp_err_t UART_SerialPrint_lcd(uint8_t *p_msg)
{
    fsp_err_t bIsError   = FSP_SUCCESS;
    uint8_t msg_len = RESET_VALUE;
    uint32_t local_timeout = (DATA_LENGTH * UINT16_MAX);
    char *p_temp_ptr = (char *)p_msg;

    /* Calculate length of message received */
    msg_len = ((uint8_t)(strlen(p_temp_ptr)));

    /* Reset callback capture variable */
    bUartEvent = RESET_VALUE;

    /* Writing to terminal */
    bIsError = R_SCI_B_UART_Write (&g_uart9_ctrl, p_msg, msg_len);


    /* Check for event transfer complete */
    while ((UART_EVENT_TX_COMPLETE != bUartEvent) && (--local_timeout))
    {
        /* Check if any error event occurred */
        if (UART_ERROR_EVENTS == bUartEvent)
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
void UART_IRQHandler9(uart_callback_args_t * p_args)
{
    /* Logged the event in global variable */
    bUartEvent = (uint8_t)p_args->event;

    /* Reset bRecivedFrame index if it exceeds than buffer size */
    if(DATA_LENGTH == bCounter)
    {
        bCounter = RESET_VALUE;
    }

    if(UART_EVENT_RX_CHAR == p_args->event)
    {
        switch (p_args->data)
        {
            /* If Enter is pressed by user, set flag to process the data */
            case CARRIAGE_ASCII:
            {
                bCounter = RESET_VALUE;
                bData_ReceivedFlag  = true;
                break;
            }
            /* Read all data provided by user until enter button is pressed */
            default:
            {
                bRecFrame[bCounter++] = (uint8_t ) p_args->data;
                break;
            }
        }
    }
}

/**
  * @brief  Verifyinh the send and received buffer while testing
  * @param  None
  * @return Success or Fail return
  */
bool UART_SerialPrint_lcd_Verify(void)
{
    // Send serial test in loop
    UART_SerialPrint_lcd((uint8_t *)TestingFrame);
    R_BSP_SoftwareDelay (100,BSP_DELAY_UNITS_MICROSECONDS);
    for(int i = 0; i < 11; i++ )
    {
        if (TestingFrame[i]!=bRecFrame[i])
            return false;
    }
    return true;
}
