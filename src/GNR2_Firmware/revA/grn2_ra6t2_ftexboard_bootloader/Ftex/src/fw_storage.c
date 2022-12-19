/**
  ******************************************************************************
  * @file    fw_storage.c
  * @brief   This file contain the firmware storage file.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "fw_storage.h"
#include "external_flash.h"
#include "littlefs_port.h"

// ================================ Defines =============================== //


// ======================= LittleFS Definition ============================= // 

lfs_t      lfs; /* LittleFS Object */
lfs_file_t file; /* File Object used by LittlsFS */
#define DFU_FILE_NAME       "dfu_file.pack"

static uint8_t lfs_file_buffer[LFS_CACHE_SIZE];  /* Static file buffer used by the LittlsFS library */ 
static uint8_t lfs_read_buffer[LFS_CACHE_SIZE];  /* Static read buffer used by the LittlsFS library */
static uint8_t lfs_prog_buffer[LFS_CACHE_SIZE];  /* Static write buffer used by the LittlsFS library */
static uint8_t lfs_lookahead_buffer[LFS_LOOKAHEAD_SIZE]; /* Static util buffer used by the LittlsFS library */

const struct lfs_file_config lfs_file_cfg = 
{
    .buffer = &lfs_file_buffer,
};

/* configuration of the LittleFS filesystem is provided by this struct */
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
OTA_Header_t hdr = {0};

// ==================== Public function prototypes ======================== //

/**
	Initializes firmware storage 
*/
uint8_t FW_StorageInit(void)
{
	uint8_t err;
	
	if(m_state == STORAGE_IDLE)
	{
		return EXIT_SUCCESS;
	}
	
	if(ExternalFash_Init() != FLASH_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{	
        err = lfs_mount(&lfs, &lfs_cfg);
        m_state = STORAGE_IDLE;
        if(err)
        {
            return EXIT_FAILURE;
        }
        else
        {
            return EXIT_SUCCESS;
        }
    }	
}

/**
	Deinitializes firmware storage 
*/
uint8_t FW_StorageDeinit(void)
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
		err = lfs_unmount(&lfs);
		if(err != LFS_ERR_OK)
		{
			return EXIT_FAILURE;
		}		
		m_state = STORAGE_UNINIT;
		return EXIT_SUCCESS;
	}	
}

/**
	Start read firmware storage pack storage 
*/
uint8_t FW_Storage_StartPackRead()
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
	
	err = lfs_file_read(&lfs, &file, hdr.Header, HEADER_SIZE);
	
	if(err != HEADER_SIZE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		m_pack_size = hdr.data.GNR_FwSize + HEADER_SIZE;
		m_state = STORAGE_PACK_READ;
		
		lfs_file_seek(&lfs, &file, hdr.data.REN_FwSize + hdr.data.BLE_FwSize + HEADER_SIZE, LFS_SEEK_SET);
		return EXIT_SUCCESS;
	}
}

/**
	Get pack chunck for firmware storage file
*/
int16_t FW_Storage_GetPackChunk(uint8_t* data, uint8_t size)
{
	uint8_t err;
	volatile uint32_t remaining_data = m_pack_size - m_file_position;
	if(m_state != STORAGE_PACK_READ)
	{
		return 0;
	}
	if(remaining_data == 0)
	{
		return 0;
	}
	
	if(remaining_data < size)
	{
		size = remaining_data;
	}
	
	err = lfs_file_read(&lfs, &file, data, size);
	
	if(err != size)
	{
		return EXIT_FAILURE;
	}
	else
	{
		m_file_position += size;
		return size;
	}
}

/**
	Finalize firmware storage file read
*/
uint8_t FW_Storage_FinalizePackRead()
{
	uint8_t err;
	
	if(m_state != STORAGE_PACK_READ)
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

/**
	Firmware storage get fw version
*/
uint32_t FW_Storage_GetGNR_FWVersion(void)
{
    return(hdr.data.GNR_FwVersion);
}

/**
	Firmware storage get fw size
*/
uint32_t FW_Storage_GetGNR_FWSize(void)
{
    return(hdr.data.GNR_FwSize);
}