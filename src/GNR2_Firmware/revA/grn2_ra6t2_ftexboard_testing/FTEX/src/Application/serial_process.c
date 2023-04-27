/**
  ******************************************************************************
  * @file    serial_process.c
  * @brief   This file contain the Serial Communication bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "serial_process.h"

// ========================== Public Variables ============================ //

extern uint8_t g_temp_buffer[DATA_LENGTH];
extern volatile uint8_t g_data_received_flag;

// ==================== Public function prototypes ======================== //

/**
  * @brief  Launch the process of the received frame
  * @param  None
  * @return None
  */
void Serial_Process_Launch (void)
{
    switch(g_temp_buffer[PROCESSTYPE])
    {
        // Command buffer process
        case COMMAND:
            if ((SF_format() == EXIT_SUCCESS))
            {
                UART_SerialPrint((uint8_t *)"Flash Format Done\r\n");
            }
            else
            {
                UART_SerialPrint((uint8_t *)"Flash Fail to Format \r\n");
            }
            break;
        // Set buffer process
        case SET:
            break;
        
        // Get buffer process
        case GET:
            break;
        
        default:
            break;
    }
}

/**
  * @brief  Process the received uart frame
  * @param  None
  * @return Upon successful open and start of timer
  */

void Serial_Frame_Process(void)
{
    if(g_data_received_flag)
    {
        g_data_received_flag  = false;

        uint8_t input_length = RESET_VALUE;

        /* Calculate bFrame_Buffer length */
        input_length = ((uint8_t)(strlen((char *) &g_temp_buffer)));
        
        /* Check if input data length is valid */
        if ((g_temp_buffer[STARTBYTE] == START ) && (g_temp_buffer[input_length-1] == CHECKFRAME ))
        {
            Serial_Process_Launch();
        }
        else
        {
            UART_SerialPrint((uint8_t *)"Data Frame Unvalid \r\n");
        }
    } 
}

