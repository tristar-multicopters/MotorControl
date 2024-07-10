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
#define DFU_FILE_NAME       "DFU_File.pack"

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

//stores computed GNR firmware size
uint32_t m_gnr_size = 0;

//used to hold the DFU pack size(except the 4 crc 
//bytes added by the GNR when compying the received
//dfu to a external memory)
uint32_t m_dfu_pack_size = 0;
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
        m_pack_size = hdr.data.gnr_fw_size + HEADER_SIZE;
        m_state = STORAGE_PACK_READ;
        
        //get DFU pack size(STM32 + renesas + GNR)
        m_dfu_pack_size = hdr.data.ble_fw_size + hdr.data.ren_fw_size + hdr.data.gnr_fw_size + HEADER_SIZE;
        
        //get gnr firmware size
        m_gnr_size = hdr.data.gnr_fw_size;
        
        //set the file position to where the GNR firmware begin.
        lfs_file_seek(&lfs, &file, hdr.data.ren_fw_size + hdr.data.ble_fw_size + HEADER_SIZE, LFS_SEEK_SET);
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
uint32_t FW_Storage_GetPack_FWVersion(void)
{
    return(hdr.data.pack_fw_version.vers_concat);
}

/**
    Firmware storage get fw size
*/
uint32_t FW_Storage_GetPack_FWSize(void)
{
    return(hdr.data.gnr_fw_size);
}

/**
    Function used to set the file position to the beginning.
    and initialize the variable m_file_position to zero.
*/
uint8_t FW_Storage_SetFileToBeginning(void)
{
    lfs_soff_t err;
    
    //set the file to his beginning.
    err = lfs_file_seek(&lfs, &file, 0, LFS_SEEK_SET);
    
    //initialize the variable to allow the function
    m_file_position = 0;
    
    //check if the file was correctly set to the beginning.
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
    function used to read size bytes from the *file
    read from the beginning of the file and moves
    size bytes to next position when one read is
    executed.
    To use this function m_file_position variable 
    must be initalized to zero before start the first 
    read.
*/
int16_t FW_Storage_ReadBytesFromFile(uint8_t* data, uint8_t size)
{
    //used to get errors when trying to execute a read operation.
    uint8_t err;
    
    //get how my bytes remain to be read from the dfu pack file.
    volatile uint32_t remaining_data = m_dfu_pack_size - m_file_position;
    
    //check if the file is in the read state.
    if(m_state != STORAGE_PACK_READ)
    {
        return 0;
    }
    
    //check if the file was completely read.
    if(remaining_data == 0)
    {
        return 0;
    }
    
    //if size required is less than remain
    //bytes, size become the reaming dara.
    if(remaining_data < size)
    {
        size = remaining_data;
    }
    
    //read size bytes from the file.
    err = lfs_file_read(&lfs, &file, data, size);
    
    //if no error increment m_file_position 
    //and return size.
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
    function used to read size bytes from the *file.
*/
int16_t FW_Storage_ReadBytes(uint8_t* data, uint8_t size)
{
    //used to get erros when trying to execute a read operation.
    uint8_t err;
    
    //check if the file is in the read state.
    if(m_state != STORAGE_PACK_READ)
    {
        return 0;
    }
	
    //read size bytes from the file.
    err = lfs_file_read(&lfs, &file, data, size);
    
    //if no error increment m_file_position 
    //and return size.
    if(err != size)
    {
        return EXIT_FAILURE;
    }
    else
    {
        return size;
    }
}

/**
    Function used to set the file position to the beginning
    of the GNR firmware at the external memory
    and initialize the variable m_file_position to zero.
*/
void FW_Storage_SetFileToGnrFirmw(void)
{
    //set the counter variable to the righ position
    m_file_position = m_dfu_pack_size - m_gnr_size;
    
    //set the file position to where the GNR firmware begin.
    lfs_file_seek(&lfs, &file, hdr.data.ren_fw_size + hdr.data.ble_fw_size + HEADER_SIZE, LFS_SEEK_SET);
}

