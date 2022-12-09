/**
  * @file    fw_storage.c
  * @brief   This file contaisn firmware storage layer.
  */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "fw_storage.h"
#include "serial_flash.h"
#include "littlefs_port.h"

/* Defines ------------------------------------------------------------------*/ 

#define HEADER_START_ADDR		0x00000000
#define FW_START_ADDR				(HEADER_START_ADDR + HEADER_SIZE)
#define FW_ADDR(n)					(FW_START_ADDR + n)

#define DFU_FILE_NAME				"dfu_file.pack"

typedef enum {
	STORAGE_UNINIT,
	STORAGE_IDLE,
	STORAGE_PACK_WRITE,
	STORAGE_PACK_READ
} storage_state_t;

/* Variables -----------------------------------------------------------------*/ 

/* LittleFS Variables */
// LittleFS Object
lfs_t lfs;
// File Object used by LittlsFS
lfs_file_t file;

// Static file buffer used by the LittlsFS library
static uint8_t lfs_file_buffer[LFS_CACHE_SIZE] __attribute__ ((aligned (32))); 
// Static read buffer used by the LittlsFS library
static uint8_t lfs_read_buffer[LFS_CACHE_SIZE] __attribute__ ((aligned (32))); 
// Static write buffer used by the LittlsFS library
static uint8_t lfs_prog_buffer[LFS_CACHE_SIZE] __attribute__ ((aligned (32)));
// Static util buffer used by the LittlsFS library
static uint8_t lfs_lookahead_buffer[LFS_LOOKAHEAD_SIZE] __attribute__ ((aligned (32)));

const struct lfs_file_config lfs_file_cfg = {
    .buffer = &lfs_file_buffer,
};

// configuration of the LittleFS filesystem is provided by this struct
const struct lfs_config lfs_cfg = {
	// block device operations
	.read  = lfs_read,
	.prog  = lfs_prog,
	.erase = lfs_erase,
	.sync  = lfs_sync,

	// block device configuration
	.read_size = LFS_READ_SIZE,
	.prog_size = LFS_PROG_SIZE,
	.block_size = LFS_BLOCK_SIZE,
	.block_count = LFS_BLOCK_COUNT,
	.cache_size = LFS_CACHE_SIZE,
	.lookahead_size = LFS_LOOKAHEAD_SIZE,
	.block_cycles = LFS_BLOCK_CYCLES,

	// buffers
	.read_buffer = &lfs_read_buffer,
	.prog_buffer = &lfs_prog_buffer,
	.lookahead_buffer = &lfs_lookahead_buffer,
};

/* Firmware Storage Variables */
// Stores current file position
uint32_t m_file_position = 0;
// Stores computed file size
uint32_t m_pack_size = 0;
// Used to keep track of current file usage
storage_state_t m_state = STORAGE_UNINIT;

// Copy of current file header
ota_header_t hdr = {0};

/* Public Functions ----------------------------------------------------------*/


uint8_t fw_storage_init( void )
{
	uint8_t err;
	
	if(m_state == STORAGE_IDLE)
	{
		return EXIT_SUCCESS;
	}
	
	if(BSP_SERIAL_FLASH_Init() != FLASH_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{
		err = lfs_mount( &lfs, &lfs_cfg );		// Getting error here itself
		if( err != LFS_ERR_OK )
		{
			BSP_SERIAL_FLASH_EraseChip();
			
			err = lfs_format( &lfs, &lfs_cfg );
			if( err != LFS_ERR_OK )
			{
				LFS_DEBUG("Error while format external flash.\n");
				//return FLASH_ERROR;
			}
			
			err = lfs_mount( &lfs, &lfs_cfg );
			if( err != LFS_ERR_OK )
			{
				LFS_DEBUG("Error while format external flash.\n");
				//return FLASH_ERROR;
			}
		}		
		m_state = STORAGE_IDLE;
		return EXIT_SUCCESS;
	}	
}

uint8_t fw_storage_deinit( void )
{
	uint8_t err;
	
	if(m_state == STORAGE_UNINIT)
	{
		return EXIT_SUCCESS;
	}
	
	if(m_state != STORAGE_IDLE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		err = lfs_unmount( &lfs );		// Getting error here itself
		if( err != LFS_ERR_OK )
		{
			return EXIT_FAILURE;
		}		
		m_state = STORAGE_UNINIT;
		return EXIT_SUCCESS;
	}	
}

// open the dfu file fro writing process
uint8_t fw_storage_start_pack_write( void )
{
	uint8_t err;
	
	if(m_state != STORAGE_IDLE)
	{
		return EXIT_FAILURE;
	}
	
	// Open and overwrite fw pack file
	err = lfs_file_opencfg(&lfs, &file, DFU_FILE_NAME, LFS_O_RDWR | LFS_O_CREAT, &lfs_file_cfg);
	
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{
		m_state = STORAGE_PACK_WRITE;
		return EXIT_SUCCESS;
	}
}
// Write a chunk of data
uint8_t fw_storage_put_pack_chunk( uint8_t* data, uint32_t size )
{
	uint8_t err;
	
	if(m_state != STORAGE_PACK_WRITE)
	{
		return EXIT_FAILURE;
	}
	
	err = lfs_file_write(&lfs, &file, data, size);
	
	if(err != size)
	{
		return EXIT_FAILURE;
	}
	else
	{
		return EXIT_SUCCESS;
	}
}

// close the file
uint8_t fw_storage_finalize_pack_write()
{
	uint8_t err;
	
	if(m_state != STORAGE_PACK_WRITE)
	{
		return EXIT_FAILURE;
	}
	
	// close fw pack file
	err = lfs_file_close(&lfs, &file);
	
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{
		m_state = STORAGE_IDLE;
		return EXIT_SUCCESS;
	}
}
// open, read header and rewind to the start of the file (dfu pack)
uint8_t fw_storage_start_pack_read()
{
	uint8_t err;
	
	if(m_state != STORAGE_IDLE)
	{
		return EXIT_FAILURE;
	}
	
	m_file_position = 0;
	
	err = lfs_file_opencfg(&lfs, &file, DFU_FILE_NAME, LFS_O_RDONLY, &lfs_file_cfg);
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	
	err = lfs_file_read(&lfs, &file, hdr.header, HEADER_SIZE);
	
	if(err != HEADER_SIZE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		m_pack_size = hdr.data.ble_fw_size + hdr.data.ren_fw_size + hdr.data.gnr_fw_size + HEADER_SIZE;
		
		m_state = STORAGE_PACK_READ;
	}
	
	err = lfs_file_rewind(&lfs, &file);
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{		
		return EXIT_SUCCESS;
	}
}
// grab a chunck of data from dfu pack file
uint8_t fw_storage_get_pack_chunk( uint8_t* data, uint8_t n )
{
	uint8_t err;
	
	if(m_state != STORAGE_PACK_READ)
	{
		return 0;
	}
	
	volatile uint32_t remaining_data = m_pack_size - m_file_position;
	if(remaining_data == 0)
	{
		return 0;
	}
	
	if(remaining_data < n)
	{
		n = remaining_data;
	}
	
	err = lfs_file_read(&lfs, &file, data, n);
	
	if(err != n)
	{
		return EXIT_FAILURE;
	}
	else
	{
		m_file_position += n;
		return n;
	}
}

// close the dfu pack file
uint8_t fw_storage_finalize_pack_read()
{
	uint8_t err;
	
	if( m_state != STORAGE_PACK_READ)
	{
		return EXIT_FAILURE;
	}
	m_state = STORAGE_IDLE;
	
	err = lfs_file_close(&lfs, &file);
	
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{
		return EXIT_SUCCESS;
	}
}

// Erase dfu file
uint8_t fw_storage_erase()
{
	uint8_t err;
	
	if( m_state != STORAGE_IDLE)
	{
		return EXIT_FAILURE;
	}
	
	err = lfs_remove(&lfs, DFU_FILE_NAME);
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}