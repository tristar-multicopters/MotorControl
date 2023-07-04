/**
  ******************************************************************************
  * @file    fw_storage.h
  * @brief   This file is the fw storage header
  ******************************************************************************
*/

#ifndef __FW_STORAGE_H
#define __FW_STORAGE_H

// =============================== Includes ================================ //
#include "stdint.h"
#include "stdlib.h"

// ================================ Defines =============================== //
/* 4*FW vers (4 bytes) + 3 * Fw size (4bytes) + 4 * Enc Key (32bytes) */
#define HEADER_SIZE     (uint32_t)156
#define HEADER_START_ADDR   0x00000000
#define FW_START_ADDR       (HEADER_START_ADDR + HEADER_SIZE)
#define FW_ADDR(n)          (FW_START_ADDR + n)
/* Structure of the DFU File header, used to easyli parse if from a raw memory bulk read */

typedef enum {
	STORAGE_UNINIT,     // Uninit firmware pack receive
	STORAGE_IDLE,       // Idle firmware pack read
	STORAGE_PACK_WRITE, // Writing the firmware pack receive
	STORAGE_PACK_READ   // Reading the firmware pack receive
} storage_state_t;

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

// ==================== Public function prototypes ======================== //

/**
* @brief  Initializes the serial flash storage
* @param  EFlash_Storage_Handle_t : serial flash storage handler
* @retval Serial Flash Storage Status
*/
uint8_t FW_StorageInit(void);

/**
* @brief  Deinitializes the serial flash storage
* @param  EFlash_Storage_Handle_t : serial flash storage handler
* @retval Serial Flash Storage Status
*/
uint8_t FW_StorageDeinit(void);

/**
* @brief  Open, Read the Header and rewind to the start of the file   
* @param  None
* @retval File Opened without error or Could not open file
*/
uint8_t FW_Storage_StartPackRead();

/**
* @brief  Write Firmware Update Data
* @param  data  pointer to the array of data to write
* @param  size  number of bytes to write
* @retval the number of character effectively written into file
*/
int16_t FW_Storage_GetPackChunk(uint8_t* data, uint8_t size);

/**
* @brief  Close Firmware Update File For writing Process
* @param  None
* @retval File closed without issues or Could not close file
*/
uint8_t FW_Storage_FinalizePackRead();

/**
* @brief  Firmware storage get fw version
* @param  None
* @retval None
*/
uint32_t FW_Storage_GetGNR_FWVersion(void);

/**
* @brief  Firmware storage get fw size
* @param  None
* @retval None    
*/
uint32_t FW_Storage_GetGNR_FWSize(void);

/**
* @brief Function used to set the file position to the beginning.
* @retval true if the file was set correctly, false if not.
*/
uint8_t FW_Storage_SetFileToBeginning(void);

/**
* @brief function used to read size bytes from the *file
         read from the beginning of the file and moves
         size bytes to next position when one read is
         executed.
         To use this function m_file_position variable 
         must be initalized to zero before start the first 
         read.
* @param  data  pointer to the array of data to write
* @param  size  number of bytes to write
* @retval the number of character effectively written into file
*/
int16_t FW_Storage_ReadBytesFromFile(uint8_t* data, uint8_t size);

/**
* @brief function used to read size bytes from the *file.
* @param  data  pointer to the array of data to write
* @param  size  number of bytes to write
* @retval the number of character effectively written into file
*/
int16_t FW_Storage_ReadBytes(uint8_t* data, uint8_t size);

/**
* @brief Function used to set the file position to the beginning
         of the GNR firmware at the external memory
         and initialize the variable m_file_position to zero.
* @retval none.
*/
void FW_Storage_SetFileToGnrFirmw(void);

#endif /* __FW_STORAGE_H */