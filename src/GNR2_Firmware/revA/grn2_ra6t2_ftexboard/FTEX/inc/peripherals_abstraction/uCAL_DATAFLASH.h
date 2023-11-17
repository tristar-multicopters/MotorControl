/**
* @file   uCal_DataFlash.h
* @author Bruno Alves
* @brief  uController Abstraction Layer for Flash memory(data and code).
*
* This module is the interface used to interact with the data flash
* memory. 
*
*/

#ifndef UCAL_DATAFLASH_H_
#define UCAL_DATAFLASH_H_

/*********************************************
               Includes                       
*********************************************/

#include "r_flash_hp.h"
#include "hal_data.h"

/*********************************************
                Defines
*********************************************/
                                     
#define DATA_FLASH_START_ADDRESS  	 0x08000000U
#define DATA_FLASH_END_ADDRESS       0x08003FFFU
#define DATA_FLASH_LAST_BLOCK_ADDRESS 0x8003FC0U
#define DATA_FLASH_BLOCK_OFFSET_64BYTES   0x00000040U

#define FLASH_HP_DF_BLOCK_0   0x08000000U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_1   0x08000040U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_2   0x08000080U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_3   0x080000C0U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_4   0x08000100U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_5   0x08000140U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_6   0x08000180U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_7   0x080001C0U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_8   0x08000200U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_9   0x08000240U //64 bytes in each block
#define FLASH_HP_DF_BLOCK_10  0x08000280U //64 bytes in each block


/*********************************************
                Data Struct Definition
*********************************************/


//struct created to handle with all necessary parameters of 
//the uCal_Data_Flash library.
typedef struct
{
    const flash_instance_t * pFlashInstance;
	  flash_result_t blank_check_result;
	  bool dataFlashOpenFlag;
    //void* Super;                     
    //void (*pFlashCallback)(void *);
}
DataFlash_Handle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief Function to open the flash memory access.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @return bool return true if operation was a success.
*/
bool uCAL_Flash_Open(DataFlash_Handle_t * pHandle);

/**
  @brief Function to close the flash memory access.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @return bool return true if operation was a success.
*/
bool uCAL_Flash_Close(DataFlash_Handle_t * pHandle);

/**
  @brief Function to erase the data flash memory.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint32_t const blockAddress start block address to be erased. 
  @oaram uint32_t const numBlocks number of black to be erased.
  @return bool return true if the data memory was correctly reased and false if not
  or a wrong parameter was passed.
*/
bool uCAL_Data_Flash_Erase(DataFlash_Handle_t * pHandle, uint32_t const blockAddress, uint32_t const numBlocks);

/**
  @brief Function to write bytes in the data flash memory.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint8_t * data pointer to the data to be written in the data flash memory.
  @param uint32_t const flashAddress start address to be written. 
  @param uint32_t const numBytes number of bytes to be written.
  @return bool return true if the data was correctly written to memory and false if not
  or a wrong parameter was passed.
*/
bool uCAL_Data_Flash_Write(DataFlash_Handle_t * pHandle, uint8_t * data, uint32_t flashAddress, uint32_t const numBytes);

/**
  @brief Function to read bytes in the data flash memory.
  To read bytes from data flash memory, data flash access
  must to be on open state.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint8_t * data pointer to the buffer that will receive data 
  from the data flash memory.
  @param uint32_t const flashAddress start address to be read. 
  @param uint32_t const numBytes number of bytes to be read.
  @return bool true if the data was read and false if not.
*/
bool uCAL_Data_Flash_Read(DataFlash_Handle_t * pHandle, uint8_t * data, uint32_t flashAddress, uint32_t const numBytes);

/**
  @brief Function to performe a blank check in the data flash memory.
  Blanck check is used to verify if the data flash contents was correctly 
  erased(0xFF). Read data from the data flash give a randon value each time 
  you make the read, if the region was erased but no programed(write operation).
  So, the only way to verify if memory was erased if calling the blank check 
  function.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint32_t const blockAddress start block address to be blank checked. 
  @oaram uint32_t const numBytes number of bytes to be blank checked(at least 4 bytes).
  @return bool return true if the memory was correctly erased and false if not
  or a wrong parameter was passed.
*/
bool uCAL_Data_Flash_Blank_Check(DataFlash_Handle_t * pHandle, uint32_t const blockAddress, uint32_t numBytes);

#endif