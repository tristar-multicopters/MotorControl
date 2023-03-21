/**
  ******************************************************************************
  * @file    serial_flash_stoarage.h
  * @author  FTEX inc
  * @brief   Header to serial flash storage 
  *          This module is the interface that is used to interact with the 
  *          external SPI Memory for pack/file storage
  ******************************************************************************
*/

#ifndef __SERIAL_FLASH_STORAGE_H
#define __SERIAL_FLASH_STORAGE_H

// =============================== Includes ================================= //
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "serial_flash.h"

// =============================== Defines ================================= //
/* 4*FW vers (2 bytes) + 3 * Fw size (4bytes) + 4 * Enc Key (32bytes) */
#define HEADER_SIZE					156
#define HEADER_START_ADDR       0x00000000
#define FW_START_ADDR           (HEADER_START_ADDR + HEADER_SIZE)
#define FW_ADDR(n)              (FW_START_ADDR + n)

// ================= Structure used to configure a pin ===================== //
// State related to the file system and file manipulation, used to avoid bad sequences of action
typedef enum {
	STORAGE_UNINIT,
	STORAGE_IDLE,
	STORAGE_PACK_WRITE,
	STORAGE_PACK_READ
} Storage_State_t;

typedef struct
{
    EFlash_Handle_t  eFlashStorage;
} EFlash_Storage_Handle_t;

// Structure of the DFU File header, used to easyli parse if from a raw memory bulk read.
typedef union {
	struct {
		uint32_t Pack_FwVersion;
		uint32_t BLE_FwVersion;
		uint32_t REN_FwVersion;
		uint32_t GNR_FwVersion;
		uint32_t BLE_FwSize;
		uint32_t REN_FwSize;
		uint32_t GNR_FwSize;
		uint8_t Pack_EncKey[32];
		uint8_t BLE_EncKey[32];
		uint8_t REN_EncKey[32];
		uint8_t GNR_EncKey[32];
	} data;
	uint8_t Header[HEADER_SIZE];
} OTA_Header_t;
// ==================== Public function prototypes ========================= //

/**
* @brief  Initializes the serial flash storage
* @param  EFlash_Storage_Handle_t : serial flash storage handler
* @retval Serial Flash Storage Status
*/
uint8_t SF_Storage_Init(EFlash_Storage_Handle_t * pHandle);

/**
* @brief  Deinitializes the serial flash storage
* @param  EFlash_Storage_Handle_t : serial flash storage handler
* @retval Serial Flash Storage Status
*/
uint8_t SF_Storage_Deinit();

/**
* @brief  Open DFU File For writing Process
* @param  None
* @retval File Created/Opened without error or Could not open file
*/
uint8_t SF_Storage_StartPackWrite();
     
/**
* @brief  Write DFU Data
* @param  data  pointer to the array of data to write
* @param  size  number of bytes to write
* @retval the number of character effectively written into file
*/
uint8_t SF_Storage_PutPackChunk(uint8_t* data, uint8_t size);

/**
* @brief  Close DFU File For writing Process
* @param  None
* @retval File closed without issues or Could not close file
*/
uint8_t SF_Storage_FinalizePackWrite();

/**
* @brief  Open, Read the Header and rewind to the start of the file   
* @param  None
* @retval File Opened without error or Could not open file
*/
uint8_t SF_Storage_StartPackRead();

/**
* @brief  Get a chunck of data from DFU Pack File 
* @param  data  pointer to the array of data to write
* @retval the number of character effectively read from file
*/
uint8_t SF_Storage_GetPackChunk( uint8_t* data, uint8_t n );

/**
* @brief  Close the DFU Pack File
* @param  None
* @retval File closed without issues or Could not close file
*/
uint8_t SF_Storage_FinalizePackRead(void);

/**
* @brief  Erase DFU File 
* @param  None
* @retval File erased without issues or Could not erase file
*/
int SF_Storage_Erase();

/**
*  @brief Return the current position fo the file.
*  @param  None
*  @retval file position.
*/
int32_t SF_STORAGE_GetCurrentFilePosition();

/**
* @brief  Read n data from the file on STORAGE_PACK_WRITE(read and write)from the 
          position.
* @param  data  pointer to the array of data to write
* @param  number of bytes to be read.
* @param  position where data will be read.
* @param  true if the data wuill be read from the beginning or false if not.
* @retval the number of character effectively read from file
*/
uint8_t SF_Storage_Read(uint8_t* data, uint8_t n , int position, bool beginning);

/**
* @brief  Open File on read only mode.
* @param  None
* @retval File Opened without error or Could not open file
*/
uint8_t SF_Storage_OpenRead(void);

/**
* @brief  Close File that was on read only mode.
* @param  None
* @retval File Close without error or Could not close file
*/
uint8_t SF_Storage_CloseRead(void);

/**
* @brief  Get the size file. File must to be open.
          Used only on read only mode.
* @param  None
* @retval Return size file or a negative number to indicate an error.
*/
int32_t SF_Storage_GetSize(void);

/**
* @brief  Move to teh end of the current file.
          Used only on read only mode.
* @param  None
* @retval Return sucess or fail.
*/
uint8_t SF_Storage_MovetoTheEndOfFile(void);

/**
   Read n data from the file on STORAGE_PACK_WRITE(read and write)from the 
   position.
*/
uint8_t SF_Storage_SetPosition(int32_t position);
#endif // __SERIAL_FLASH_STORAGE_H