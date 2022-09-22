/**
  ******************************************************************************
  * @file    serial_flash.c
  * @author  FTEX inc
  * @brief   module to handle firmware pack/file manipulation via an
	*					 external spi flash memory
  ******************************************************************************
*/


#ifndef __LITTLEFS_PORT_H
#define __LITTLEFS_PORT_H

// =============================== Includes ================================= //
#include <stdint.h>
//#include "lfs.h"
#include "serial_flash.h"
#include "MX25L3233F_driver.h"

// =============================== Defines ================================= //
#define LFS_BLOCK_SIZE                  MX25L3233F_SECTOR_4K
#define LFS_BLOCK_COUNT                 (MX25L3233F_FLASH_SIZE / LFS_BLOCK_SIZE)
#define LFS_READ_SIZE                   256
#define LFS_PROG_SIZE                   256
#define LFS_CACHE_SIZE                  256
#define LFS_LOOKAHEAD_SIZE              256
#define LFS_BLOCK_CYCLES                500

// ================= Structure used to configure a pin ===================== //

EFlash_Handle_t * LfsHandle;

// ==================== Public function prototypes ========================= //
/**
  * @brief  Littlefs file system read
  * @param  
  * @retval 
  */
//int lfs_read(const struct lfs_config *c, lfs_block_t block,
            //lfs_off_t off, void *buffer, lfs_size_t size);

/**
  * @brief  Littlefs file system program
  * @param  
  * @retval 
  */
//int lfs_prog(const struct lfs_config *c, lfs_block_t block,
            //lfs_off_t off, const void *buffer, lfs_size_t size);
/**
	* @brief Littlefs file system erase
  * @param  
  * @retval
*/
//int lfs_erase(const struct lfs_config *c, lfs_block_t block);

/**
  * @brief  Littlefs file system synchronize
  * @param  
  * @retval 
  */
//int lfs_sync(const struct lfs_config *c);

#endif // __LITTLEFS_PORT_H