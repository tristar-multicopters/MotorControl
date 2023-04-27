/**
  ******************************************************************************
  * @file    Main.h
  * @brief   This file contain the header for main firmware application
  ******************************************************************************
*/
#ifndef __TEST_MAIN_H
#define __TEST_MAIN_H

// =============================== Includes ================================ //
#include "hal_data.h"
#include "serial_communication.h"
#include "external_flash_storage.h"
#include "external_memory_spi.h"
#include "serial_process.h"

// ================================ Defines =============================== //


// ==================== Public function prototypes ======================== //

/**
    Initializes peripheral used by Serial Communication UART.
*/
void Test_Main(void);
    

#endif /* __TEST_MAIN_H */