/**
  * @file    fw_storage.h
  * @brief   This file is the device firmware storage header
	*/

#ifndef __FW_STORAGE_H
#define __FW_STORAGE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Defines --------------------------------------------------------------------*/

// 4*FW vers (2 bytes) + 3 * Fw size (4bytes) + 4 * Enc Key (32bytes)
#define HEADER_SIZE					156

// Structure of the DFU File header, used to easyli parse if from a raw memory bulk read.
typedef union {
	struct {
		uint32_t pack_fw_version;
		uint32_t ble_fw_version;
		uint32_t ren_fw_version;
		uint32_t gnr_fw_version;
		uint32_t ble_fw_size;
		uint32_t ren_fw_size;
		uint32_t gnr_fw_size;
		uint8_t pack_enc_key[32];
		uint8_t stm_enc_key[32];
		uint8_t ren_enc_key[32];
		uint8_t gnr_enc_key[32];
	} data;
	uint8_t header[HEADER_SIZE];
} ota_header_t;

/* Function Prototypes ------------------------------------------------------------*/

/*
 * @brief	Initializes firware storage
 *			This function initializes the Flash Driver and mounts the file system. If file system cannot be mounted, the
 *			external flash is erased and formatted before reatempting to mount. Firmware storage should be in uninit
 *			prior to calling this function.
 * @param	nothing
 * @retval  EXIT_SUCCESS  Initialization successfull
 * @retval  EXIT_FAILURE  Could not initialize firmware storage
**/
uint8_t fw_storage_init( void );

/*
 * @brief Deinitializes firware storage
 *			This functions unmounts the file system and releases its resources. Firmware storage should be in idle state 
 *			prior to calling this function.
 * @param	nothing
 * @retval  EXIT_SUCCESS  Denitialization successfull
 * @retval  EXIT_FAILURE  Could not deinitialize firmware storage
**/
uint8_t fw_storage_deinit( void );

/*
 * @brief 	Open a file or creates it with write access rights.
 * @param	nothing
 * @retval  EXIT_SUCCESS  File Created/Opened without error
 * @retval  EXIT_FAILURE  Could not open file
**/
uint8_t fw_storage_start_pack_write( void );

/*
 * @brief Writes a chunk of data into opened file, at current file position.
 * 			The provided data chunk is writtend into currently opened file at the current file position. A call to 
 * 			fw_storage_start_pack_write should be done prior to any write attempt. Changes are effectove only after
 *			closing file with fw_storage_finalize_pack_write().
 * @param[in]	data	pointer to the array of data to write
 * @param	size	number of bytes to write
 * @return  Function returns the number of character effectively written into file
**/
uint8_t fw_storage_put_pack_chunk( uint8_t* data, uint32_t size );

/*
 * @brief Used to close a file after creating/writing to it
 * @param	nothing
 * @retval  EXIT_SUCCESS  File closed without issues
 * @retval  EXIT_FAILURE  Could not close file
**/
uint8_t fw_storage_finalize_pack_write(void);

/*
 * @brief Open a file with read access right. Read header to compute file size and rewind.
 * @param	nothing
 * @retval  EXIT_SUCCESS  File Opened without error
 * @retval  EXIT_FAILURE  Could not open file
**/
uint8_t fw_storage_start_pack_read( void );

/*
 * @brief Read a chunk of data from opened file, at current file position.
 * 		  The provided data chunk is read from currently opened file at the current file position. A call to 
 * 		  fw_storage_start_pack_read should be done prior to any read attempt.
 * @param	nothing
 * @return  Function returns the number of character effectively read from file
**/
uint8_t fw_storage_get_pack_chunk( uint8_t* data, uint8_t n );

/*
 * @brief Used to close a file after reading it
 * @param[out]	data	pointer to the buffer where read data will be stored
 * @param 	n	number of bytes to read
 * @retval  EXIT_SUCCESS File closed without issues
 * @retval  EXIT_FAILURE Could not close file
**/
uint8_t fw_storage_finalize_pack_read( void );

/*
 * @brief Erases dfu file
 * @param nothing
 * @retval  EXIT_SUCCESS File erased without issues
 * @retval  EXIT_FAILURE Could not erase file
**/
uint8_t fw_storage_erase();


#endif /* __FW_STORAGE_H */