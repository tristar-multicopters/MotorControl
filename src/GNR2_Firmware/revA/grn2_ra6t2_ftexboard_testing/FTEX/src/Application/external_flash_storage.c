/**
  ******************************************************************************
  * @file    serial_flash_storage.c
  * @author  FTEX inc
  * @brief   Serial Flash storage handle the firmware pack/file manipulation via an
  *          external SPI flash memory using a file system
  ******************************************************************************
*/
// =============================== Includes ================================= //
#include "littlefs_port.h"
#include "external_flash_storage.h"

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

// ==================== Public function prototypes ======================== //

/**
    Initializes the serial flash storage
*/
uint8_t SF_format(void)
{
    // Initialize the external Flash / File system
    if(ExternalMemory_Init() != FLASH_OK)
    {
        return false;;
    }
    // Format External flash
    if(ExternalMemory_EraseChip() != FLASH_OK)
    {
        return false;;
    }
    return true;
}