/**
  * @file    fw_storage.h
  * @brief   This file is the fw storage header
	*/

#ifndef __FW_STORAGE_H
#define __FW_STORAGE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdlib.h"

/* Defines --------------------------------------------------------------------*/
// 4*FW vers (4 bytes) + 3 * Fw size (4bytes) + 4 * Enc Key (32bytes)
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

/**
	Initializes firmware storage 
*/
uint8_t fw_storage_init( void );

/**
	Deinitializes firmware storage 
*/
uint8_t fw_storage_deinit( void );

/**
	Start read firmware storage pack storage 
*/
uint8_t fw_storage_start_pack_read();

/**
	Get pack chunck for firmware storage file
*/
int16_t fw_storage_get_pack_chunk( uint8_t* data, uint8_t n );

/**
	Finalize firmware storage file read
*/
uint8_t fw_storage_finalize_pack_read();

/**
	Firmware storage get fw version
*/
uint16_t fw_storage_get_gnr_fw_version(void);

/**
	Firmware storage get fw size
*/
uint32_t fw_storage_get_gnr_fw_size(void);




#endif /* __FW_STORAGE_H */