/**
  ******************************************************************************
  * @file    Serial_Communication.c
  * @brief   This file contain the Serial Communication bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "test_main.h"

// ==================== Public function prototypes ======================== //


/**
 * @brief Function for main application entry.
 */
void Test_Main(void)
{
    /* Initialization */
    Init_uart();
    Init_spi();
    
    /* Start task */
    while (true)
    {
        Serial_Frame_Process();
    }
    
}