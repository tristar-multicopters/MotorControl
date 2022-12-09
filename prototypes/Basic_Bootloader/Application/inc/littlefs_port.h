/**
  * @file    Littlefs_port.h
  * @brief   This file is the file system port header
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LITTLEFS_PORT_H
#define __LITTLEFS_PORT_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "lfs.h"
#include "serial_flash.h"
#include "mx25_driver.h"

/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
uint32_t address;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
#define LFS_BLOCK_SIZE      MX25L3233F_SECTOR_4K
#define LFS_BLOCK_COUNT     (MX25L3233F_FLASH_SIZE / LFS_BLOCK_SIZE)
#define LFS_READ_SIZE       16
#define LFS_PROG_SIZE       16
#define LFS_CACHE_SIZE      512
#define LFS_LOOKAHEAD_SIZE  16
#define LFS_BLOCK_CYCLES    500

/* Exported functions prototypes ---------------------------------------------*/
int lfs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size)
{
	uint8_t err;
	address = block * c->block_size + off;
	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else{
		err = BSP_SERIAL_FLASH_ReadData( address, buffer, size );
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
	
int lfs_erase(const struct lfs_config *c, lfs_block_t block)
{
	uint8_t err;
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

/* Private defines -----------------------------------------------------------*/




#endif // __LITTLEFS_PORT_H
