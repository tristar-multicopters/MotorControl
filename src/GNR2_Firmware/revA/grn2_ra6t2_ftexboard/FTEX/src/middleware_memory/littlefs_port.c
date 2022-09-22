/**
  ******************************************************************************
  * @file    serial_flash.c
  * @author  FTEX inc
  * @brief   module to handle firmware pack/file manipulation via an
	*					 external spi flash memory
  ******************************************************************************
*/
// =============================== Includes ================================= //
#include "littlefs_port.h"

// ==================== Public function prototypes ======================== //


/**
	Littlefs file system read
  */
	/*
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
		err = Serial_Flash_ReadData(LfsHandle,address, buffer, size);
		if( err == FLASH_ERROR )
		{
			return LFS_ERR_INVAL;
		}
		else
		{
			return LFS_ERR_OK;
		}
	}
};*/

/**
	Littlefs file system program
*/
/*
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
		err = Serial_Flash_WriteData(LfsHandle, address, (uint8_t*)buffer, size);
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
*/
/**
	Littlefs file system erase
*/
/*
int lfs_erase(const struct lfs_config *c, lfs_block_t block)
{
	uint8_t err;
	 // This is fine to use the sector erase function with block number as we
	 // define lfs_block size to the same value as MX25L3233F sector size.
	 // Therefore, we use littls fs to work at the sector level instead of 
	 // working at the block level

	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
		err = Serial_Flash_EraseSector(LfsHandle, block);
		if (err == FLASH_ERROR)
		{
			return LFS_ERR_INVAL;
		}
		else{
			return LFS_ERR_OK;
		}
	}
};*/

/**
	Littlefs file system synchronize 
*/
/*
int lfs_sync(const struct lfs_config *c)
{
	if(c->block_size == 0)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
		return LFS_ERR_OK;
	}
};*/