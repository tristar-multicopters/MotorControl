/**
  ******************************************************************************
  * @file    serial_flash_storage.c
  * @author  FTEX inc
  * @brief   Serial Flash storage handle the firmware pack/file manipulation via an
  *          external SPI flash memory using a file system
  ******************************************************************************
*/
// =============================== Includes ================================= //
#include "stdlib.h"
#include "littlefs_port.h"
#include "serial_flash_storage.h"

// ======================= Privates Definition ============================= //
#define HEADER_START_ADDR   0x00000000
#define FW_START_ADDR       (HEADER_START_ADDR + HEADER_SIZE)
#define FW_ADDR(n)          (FW_START_ADDR + n)

#define DFU_FILE_NAME       "DFU_File.pack"

// ======================= LittleFS Definition ============================= // 

lfs_t      lfs; /* LittleFS Object */
lfs_file_t file; /* File Object used by LittlsFS */

static uint8_t lfs_file_buffer[LFS_CACHE_SIZE] __attribute__ ((aligned (32))); /* Static file buffer used by the LittlsFS library */ 
static uint8_t lfs_read_buffer[LFS_CACHE_SIZE] __attribute__ ((aligned (32))); /* Static read buffer used by the LittlsFS library */
static uint8_t lfs_prog_buffer[LFS_CACHE_SIZE] __attribute__ ((aligned (32))); /* Static write buffer used by the LittlsFS library */
static uint8_t lfs_lookahead_buffer[LFS_LOOKAHEAD_SIZE] __attribute__ ((aligned (32))); /* Static util buffer used by the LittlsFS library */

const struct lfs_file_config lfs_file_cfg = {
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
uint32_t w_FilePosition = 0;
// Stores computed file size
uint32_t w_PackSize = 0;
// Used to keep track of current file usage
Storage_State_t m_state = STORAGE_UNINIT;

// Copy of current file header
OTA_Header_t hdr = {0};

// ==================== Public function prototypes ======================== //

/**
    Initializes the serial flash storage
*/
uint8_t SF_Storage_Init(EFlash_Storage_Handle_t * pHandle)
{
	int err;
    // Check the external Flash state
	if(m_state == STORAGE_IDLE)
	{
		return EXIT_SUCCESS;
	}
    // Initialize the external Flash / File system
	if(Serial_Flash_Init(&pHandle->eFlashStorage) != FLASH_OK)
	{
		return EXIT_FAILURE;
	}
	else
    {
        // Mount the file system 
		err = lfs_mount( &lfs, &lfs_cfg );
		if(err != LFS_ERR_OK)
		{
            // Erase the external flash if the file is not mounted
			Serial_Flash_EraseChip(&pHandle->eFlashStorage);
			// Format the external flash if the file is not mounted
			err = lfs_format(&lfs, &lfs_cfg);
			if(err != LFS_ERR_OK)
			{
				return FLASH_ERROR;
			}
			// Mount the file system 
			err = lfs_mount(&lfs, &lfs_cfg);
			if( err != LFS_ERR_OK )
			{
                // Return error for file system fail
				return FLASH_ERROR;
			}
		}		
		m_state = STORAGE_IDLE;
		return EXIT_SUCCESS;
	}		
}

/**
    Deinitializes the serial flash storage
*/
uint8_t SF_Storage_Deinit()
{
	int err;
	
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
   Open DFU File For writing Process
*/
uint8_t SF_Storage_StartPackWrite()
{
	int err;
	
	if(m_state != STORAGE_IDLE)
	{
		return EXIT_FAILURE;
	}
	
	/* Open and Overwrite Firmware Pack File */
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

/**
   Write DFU Data  
*/
uint8_t SF_Storage_PutPackChunk(uint8_t* data, uint8_t size)
{
	int err;
	
	if(m_state != STORAGE_PACK_WRITE)
	{
		return EXIT_FAILURE;
	}
	/* Write FW Pack file */
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

/**
   Close DFU File For writing Process  
*/
uint8_t SF_Storage_FinalizePackWrite()
{
	int err;
	
	if(m_state != STORAGE_PACK_WRITE)
	{
		return EXIT_FAILURE;
	}
	
	/* Close FW Pack file */
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

/**
   Open, Read the Header and rewind to the start of the file   
*/
uint8_t SF_Storage_StartPackRead()
{
	int err;
	
	if(m_state != STORAGE_IDLE)
	{
		return EXIT_FAILURE;
	}
	
	w_FilePosition = 0;
	/* Open File system for reading */
	err = lfs_file_opencfg(&lfs, &file, DFU_FILE_NAME, LFS_O_RDONLY, &lfs_file_cfg);
	if(err != LFS_ERR_OK)
	{
		return EXIT_FAILURE;
	}
	/* Read file using lfs read */
	err = lfs_file_read(&lfs, &file, hdr.Header, HEADER_SIZE);
	
	if(err != HEADER_SIZE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		w_PackSize = hdr.data.BLE_FwSize + hdr.data.REN_FwSize + hdr.data.GNR_FwSize + HEADER_SIZE;
		
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

/**
   Get a chunck of data from DFU Pack File
*/
uint8_t SF_Storage_GetPackChunk( uint8_t* data, uint8_t n )
{
	int err;
	
	if(m_state != STORAGE_PACK_READ)
	{
		return 0;
	}
	
	volatile uint32_t remaining_data = w_PackSize - w_FilePosition;
	if(remaining_data == 0)
	{
		return 0;
	}
	
	if(remaining_data < n)
	{
		n = (uint8_t) remaining_data;
	}
	
	err = lfs_file_read(&lfs, &file, data, n);
	
	if(err != n)
	{
		return EXIT_FAILURE;
	}
	else
	{
		w_FilePosition += n;
		return n;
	}
}

/**
   Close the DFU Pack File
*/
uint8_t SF_Storage_FinalizePackRead(void)
{
	int err;
	
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

/**
   Erase DFU File  
*/
uint8_t SF_Storage_Erase()
{
	int err;
	
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
