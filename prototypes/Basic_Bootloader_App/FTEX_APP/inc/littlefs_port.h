/**
  ******************************************************************************
  * @file           : littlefs_port.h
  * @brief          : The purpose of this header file is to link our 
  *                   serial_flash API with littleFS API prototype.
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LITTLEFS_PORT_H
#define __LITTLEFS_PORT_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "lfs.h"
#include "serial_flash.h"
#include "mx25l3233f.h"

/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
#define LFS_BLOCK_SIZE						MX25L3233F_SECTOR_4K
#define LFS_BLOCK_COUNT						(MX25L3233F_FLASH_SIZE / LFS_BLOCK_SIZE)
#define LFS_READ_SIZE							16
#define LFS_PROG_SIZE						  16
#define LFS_CACHE_SIZE						256
#define LFS_LOOKAHEAD_SIZE				16
#define LFS_BLOCK_CYCLES					500

/* Exported functions prototypes ---------------------------------------------*/
int lfs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size)
{
	uint8_t err;
	uint32_t address = block * c->block_size + off;
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
	uint8_t err;
	uint32_t address = block * c->block_size + off;
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
		err = BSP_SERIAL_FLASH_EraseSector( block );
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
		return LFS_ERR_OK;
	}
};

/* Private defines -----------------------------------------------------------*/




#endif // __LITTLEFS_PORT_H