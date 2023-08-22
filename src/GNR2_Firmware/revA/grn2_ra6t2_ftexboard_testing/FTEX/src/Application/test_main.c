/**
  ******************************************************************************
  * @file    Test_Main.c
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
    Init_uart_lcd();
    Init_spi();
    /* Start task */
    while (true)
    {
        /* Main process Testing */
        SerialProcess_Main();
    }
}