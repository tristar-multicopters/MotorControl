/**
  ******************************************************************************
  * @file    littlefs_port.c
  * @author  FTEX inc
  * @brief   Header to littlefs port 
  *          This module is the interface that link the file system command to 
  *          the external spi flash driver command
  ******************************************************************************
*/

#ifndef __LITTLEFS_PORT_H
#define __LITTLEFS_PORT_H

// =============================== Includes ================================= //
#include "stdint.h"
#include "lfs.h"
#include "serial_flash.h"
#include "MX25L3233F_driver.h"
#include "comm_config.h"

// =============================== Defines ================================= //
#define LFS_BLOCK_SIZE                  MX25L3233F_SECTOR_4K
#define LFS_BLOCK_COUNT                 (MX25L3233F_FLASH_SIZE / LFS_BLOCK_SIZE)
#define LFS_READ_SIZE                   16
#define LFS_PROG_SIZE                   16
#define LFS_CACHE_SIZE                  512
#define LFS_LOOKAHEAD_SIZE              16
#define LFS_BLOCK_CYCLES                500

// ==================== Public function prototypes ========================= //
/**
  * @brief  Littlefs file system read
  * @param  
  * @retval 
  */
int lfs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size);

/**
  * @brief  Littlefs file system program
  * @param  
  * @retval 
  */
int lfs_prog(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size);
/**
  * @brief Littlefs file system erase
  * @param  
  * @retval
*/
int lfs_erase(const struct lfs_config *c, lfs_block_t block);

/**
  * @brief  Littlefs file system synchronize
  * @param  
  * @retval 
  */
int lfs_sync(const struct lfs_config *c);

#endif // __LITTLEFS_PORT_H