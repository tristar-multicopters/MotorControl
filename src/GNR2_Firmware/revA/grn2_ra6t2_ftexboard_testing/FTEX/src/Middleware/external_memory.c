/**
  ******************************************************************************
  * @file    external_flash.c
  * @brief   This file contain the external flash bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "external_memory.h"

// ==================== Public function prototypes ======================== //

/**
    Initializes peripherals used by the Serial FLASH device.
*/
uint8_t ExternalMemory_Init(void)
{
    /* Flash ID testing buffer */
    uint8_t flash_id[3] = {0};
    /* External Flash software reset */
    if(SF_ResetEnable() != MX25L3233F_OK) 
        return FLASH_ERROR;

    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

    /* External Flash Memory software reset */
    if(SF_ResetMemory() != MX25L3233F_OK) 
        return FLASH_ERROR;

    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
	
    /* External Flash Read ID */
	SF_ReadID(flash_id);
	
	if(flash_id[0] != MX25L3233F_MANUFACTURER_ID)
	{
		return FLASH_ERROR;
	}
	else if(flash_id[1] != MX25L3233F_MEMORY_TYPE)
	{
		return FLASH_ERROR;
	}
	else if(flash_id[2] != MX25L3233F_MEMORY_CAPACITY)
	{
		return FLASH_ERROR;
	}
	else
	{
		return FLASH_OK;
	}
	
}
#ifndef LFS_READONLY

/**
    Erases the specified FLASH sector.
*/
uint8_t ExternalMemory_EraseSector(uint32_t SectorAddr)
{
    /* Sector Erase */
    SF_WriteEnable();

    SF_BlockErase(SectorAddr, MX25L3233F_ERASE_4K);

    /* Wait the end of Flash writing and Deselect the FLASH */
    if(SF_AutoPollingMemReady(MX25L3233F_GENERAL_TIME_OUT) != MX25L3233F_OK)
    {
        return FLASH_ERROR;
    }
    else
    {
        return FLASH_OK;
    }
}

/**
  * @brief  Erases the entire FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t ExternalMemory_EraseChip(void)
{
    /* Sector Erase */
    SF_WriteEnable();

    SF_ChipErase();

    /* Wait the end of Flash writing and Deselect the FLASH */
    if(SF_AutoPollingMemReady(MX25L3233F_CHIP_ERASE_MAX_TIME) != MX25L3233F_OK)
    {
        return FLASH_ERROR;
    }
    else
    {
        return FLASH_OK;
    }
}

/**
    Writes block of data to the FLASH. In this function, the number of
    WRITE cycles are reduced, using Page WRITE sequence.
*/
uint8_t ExternalMemory_WriteData(uint32_t WriteAddr, uint8_t* pData, uint32_t Size)
{
    uint8_t ret = FLASH_OK;
    uint32_t end_addr, current_size, current_addr;

    /* Calculation of the size between the write address and the end of the page */
    current_size = MX25L3233F_PAGE_SIZE - (WriteAddr % MX25L3233F_PAGE_SIZE);

    /* Check if the size of the data is less than the remaining place in the page */
    if (current_size > Size)
    {
        current_size = Size;
    }

    /* Initialize the address variables */
    current_addr = WriteAddr;
    end_addr = WriteAddr + Size;

    /* Perform the write page by page */
    do
    {
    /* Enable write operations */
        if(SF_WriteEnable() != MX25L3233F_OK)
        {
        ret = FLASH_ERROR;
        }/* Issue page program command */
        else if(SF_PageProgram(pData, current_addr, current_size) != MX25L3233F_OK)
        {
            ret = FLASH_ERROR;
        }/* Configure automatic polling mode to wait for end of program */
        else if(SF_AutoPollingMemReady(MX25L3233F_GENERAL_TIME_OUT) != MX25L3233F_OK)
        {
            ret = FLASH_ERROR;
        }
        /* Update the address and size variables for next page programming */
        current_addr += current_size;
        pData += current_size;
        current_size = ((current_addr + MX25L3233F_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25L3233F_PAGE_SIZE;
    } while((current_addr < end_addr) && (ret == FLASH_OK));

    /* Return BSP status */
    return ret;
}
#endif

/**
    Reads a block of data from the FLASH.
*/
uint8_t ExternalMemory_ReadData(uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize)
{
    if(SF_Read(pData, uwStartAddress, uwDataSize)!= MX25L3233F_OK)
    {
        return FLASH_ERROR;
    }
    else
    {
        return FLASH_OK;
    }
}

/**
    Reads FLASH identification.
*/
uint8_t ExternalMemory_ReadID(uint8_t* pData)
{
    if(SF_ReadID(pData) != MX25L3233F_OK)
    {
        return FLASH_ERROR;
    }
    else
    {
        return FLASH_OK;
    }
}
