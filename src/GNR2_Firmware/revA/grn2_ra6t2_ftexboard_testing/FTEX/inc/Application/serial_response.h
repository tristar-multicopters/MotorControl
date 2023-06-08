/**
  ******************************************************************************
  * @file    serial_response.h
  * @brief   This file contain the header for serial response application
  ******************************************************************************
*/
#ifndef __SERIAL_RESPONSE_H
#define __SERIAL_RESPONSE_H

// =============================== Includes ================================ //
#include "serial_communication.h"
#include "external_flash_storage.h"
#include "internal_flash_storage.h"

// ==================== Public function prototypes ======================== //

/**
  * @brief  Send Acknowledge after data verification 
  * @param  uint8_t Code_Byte
  * @return None
  */
void Serial_Acknowledge(uint8_t Code_Byte);

/**
  * @brief  Send Error after data verification 
  * @param  uint8_t Error_Byte
  * @return None
  */
void Serial_Error(uint8_t Error_Byte);
    

#endif /* __SERIAL_RESPONSE_H */