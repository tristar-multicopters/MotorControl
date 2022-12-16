/**
  ******************************************************************************
  * @file    littlefs_port.c
  * @author  FTEX inc
  * @brief   module to link the file system to the
	*		 External SPI Flash Memory
  ******************************************************************************
*/
// =============================== Includes ================================= //
#include "littlefs_port.h"

// =============================== Variables ================================= //

EFlash_Handle_t * LfsHandle;

// ==================== Public function prototypes ======================== //

/**
	Littlefs file system read link to the external flash memory
  */
int lfs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size)
{
	uint8_t err;
    // define the adress of reading
	uint32_t addresse = block * c->block_size + off;
	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else
    {   
        // Link the serial flash memory read data to the file system
		err = Serial_Flash_ReadData(LfsHandle, addresse, buffer, size);
		if(err == FLASH_ERROR)
		{
			return LFS_ERR_INVAL;
		}
		else
		{
			return LFS_ERR_OK;
		}
	}
}

/**
	Littlefs file system program link to the external flash memory
*/

int lfs_prog(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size)
{
	uint8_t err;
    // define the adress of writing 
	uint32_t addresse = block * c->block_size + off;
	if(block > c->block_count)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
        // Link the serial flash memory write data to the file system
		err = Serial_Flash_WriteData(LfsHandle, addresse, (uint8_t*)buffer, size);
		if( err == FLASH_ERROR )
		{
			return LFS_ERR_INVAL;
		}
		else
		{
			return LFS_ERR_OK;
		}
	}
}

/**
	Littlefs file system erase link to the external flash memory
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
        // Link the serial flash memory erase data to the file system
		err = Serial_Flash_EraseSector(LfsHandle, block);
		if (err == FLASH_ERROR)
		{
			return LFS_ERR_INVAL;
		}
		else{
			return LFS_ERR_OK;
		}
	}
}

/**
	Littlefs file system synchronizelink to the external flash memory 
*/
int lfs_sync(const struct lfs_config *c)
{
	if(c->block_size == 0)
	{
		return LFS_ERR_INVAL;
	}
	else
	{
		return 0;
	}
}
