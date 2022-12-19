/**
  ******************************************************************************
  * @file    Littlefs_port.h
  * @brief   This file is the file system port header
  ******************************************************************************
*/

#ifndef __LITTLEFS_PORT_H
#define __LITTLEFS_PORT_H

// =============================== Includes ================================ //
#include "stdint.h"
#include "lfs.h"
#include "external_flash.h"
#include "mx25_driver.h"

// ================================ Defines =============================== //

#define LFS_BLOCK_SIZE      MX25L3233F_SECTOR_4K
#define LFS_BLOCK_COUNT     (MX25L3233F_FLASH_SIZE / LFS_BLOCK_SIZE)
#define LFS_READ_SIZE       16
#define LFS_PROG_SIZE       16
#define LFS_CACHE_SIZE      512
#define LFS_LOOKAHEAD_SIZE  16
#define LFS_BLOCK_CYCLES    500

uint32_t address;

// ==================== Public function prototypes ======================== //

/**
  * @brief  Read a region in a memory block.
  * @param  lfs_config Configuration provided during initialization of the littlefs
  * @param  lfs_block_t File system block select
  * @param  lfs_off_t File system block size
  * @param  buffer to read during process
  * @param  lfs_size_t file system size to read
  * @retval Errorcode or 0 on success.
  */
int lfs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size)
{
	int err;
	address = block * c->block_size + off;
	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else{
		err = ExternalFash_ReadData( address, buffer, size );
		if( err == FLASH_ERROR )
		{
			return LFS_ERR_INVAL;
		}
		else
		{
			return LFS_ERR_OK;
		}
	}
};

/**
  * @brief  Program a region in a block.
  * @param  lfs_config Configuration provided during initialization of the littlefs
  * @param  lfs_block_t File system block select
  * @param  lfs_off_t File system block size
  * @param  buffer to read during process
  * @param  lfs_size_t file system size to read
  * @retval Errorcode or 0 on success.
  */
int lfs_prog(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size)
{
	#ifndef LFS_READONLY

	uint8_t err;
	address = block * c->block_size + off;
	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
		err = BSP_SERIAL_FLASH_WriteData( address, (uint8_t*)buffer, size );
		if( err == FLASH_ERROR )
		{
			return LFS_ERR_INVAL;
		}
		else
		{
			return LFS_ERR_OK;
		}
	}
	#elseif
		return LFS_ERROR;
	#endif
};

/**
  * @brief  Erase a region in a block.
  * @param  lfs_config Configuration provided during initialization of the littlefs
  * @param  lfs_block_t File system block select
  * @retval Errorcode or 0 on success.
  */
int lfs_erase(const struct lfs_config *c, lfs_block_t block)
{
	int err;
	/* 
	 * This is fine to use the sector erase function with block number as we
	 * define lfs_block size to the same value as MX25L3233F sector size.
	 * Therefore, we use littls fs to work at the sector level instead of 
	 * working at the block level
	 */
	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
		#ifdef LFS_READ_ONLY
		err = BSP_SERIAL_FLASH_EraseSector( block );
		#endif
		if (err == FLASH_ERROR)
		{
			return LFS_ERR_INVAL;
		}
		else{
			return LFS_ERR_OK;
		}
	}
};

/**
  * @brief  Sync the state of the underlying block device.
  * @param  lfs_config Configuration provided during initialization of the littlefs
  * @retval Errorcode or 0 on success.
  */
int lfs_sync( const struct lfs_config *c )
{
	if(c->block_size == 0)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
		return 0;
	}
};

#endif // __LITTLEFS_PORT_H
