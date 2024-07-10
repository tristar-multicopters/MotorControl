/**
  ******************************************************************************
  * @file    external_flash_storage.h
  * @author  FTEX inc
  * @brief   Header to external flash storage 
  *          This module is the interface that is used to interact with the 
  *          external SPI Memory
  ******************************************************************************
*/

#ifndef __EXTERNAL_FLASH_STORAGE_H
#define __EXTERNAL_FLASH_STORAGE_H

// =============================== Includes ================================= //
#include "internal_memory.h"
#include "serial_communication.h"
// =============================== Defines ================================= //


// Maximum Serial unit Length
#define SERIAL_LENGTH                   12U

// Internal Flahs blcok bytes number
#define NUMBER_OF_BYTES_IN_THE_BLOCK    64U

//first byte used to show that data memory
//has user configuration
#define ID0_DATA_FLASH                 0xB5

//second byte used to show that data memory
//has user configuration
#define ID1_DATA_FLASH                 0xC3

//definition used to control how many
//blocks of the data flash memory are being
//used to hold the user configuration.
//each block has 64 bytes.
#define NUMBER_OF_BLOCKS_USED  1

//code used to indicate that data flash
//memory(user configuration) is being updated by an external
//device.
//This value must be write in the respective 
//Object dictionary ID(OD-ID) to disable theses variables
//to be upadted by the application using UpdateObjectDictionnary().
#define KEY_USER_DATA_CONFIG_BEING_UPDATED   0xD5A3

//code used to indicate that data flash
//memory was updated by an external
//device.
//This value must be write in the respective 
//Object dictionary ID(OD-ID) to disable theses variables
//to be upadted by the application using UpdateObjectDictionnary()
//and to trigger a task responsible to write the values received 
//in the data flash memory and reset the system.
#define KEY_USER_DATA_CONFIG_UPDATED         0xC2E5

//CRC-16-CCITT polynom
#define CCITT_POLYNOM 0x1021

#define MAX_ERASE_ATTEMPTS     3

// ==================== Public function prototypes ========================= //

/**
  * @brief Function to write user data config into the data flash memory.
  * @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
           to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  * @return void
*/
void InternalMemory_WriteSerialNumber(uint8_t SaveData[SERIAL_LENGTH]);


void InternalMemory_ReadSerialNumber(void);

#endif // __INTERNAL_FLASH_STORAGE_H