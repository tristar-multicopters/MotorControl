/**
  ******************************************************************************
  * @file    serial_flash.c
  * @author  FTEX inc
  * @brief   module to manage the external Flash Memory
  ******************************************************************************
*/

// =============================== Includes ================================= //
#include "serial_flash.h"


// ==================== Public function prototypes ======================== //
/**
	Initializes peripherals used by the Serial FLASH device.
*/
uint8_t Serial_Flash_Init(EFlash_Handle_t * pHandle)
{
    uint8_t flash_id[3] = {0};
	
    if (MX25L3233F_ResetEnable(&pHandle->eFlash) != MX25L3233F_OK) 
        return FLASH_ERROR;
    osDelay(1);
    
    if (MX25L3233F_ResetMemory(&pHandle->eFlash) != MX25L3233F_OK) 
        return FLASH_ERROR;
    osDelay(1);

    Serial_Flash_ReadID(pHandle, flash_id);
	
    if (flash_id[0] != MX25L3233F_MANUFACTURER_ID)
    {
        return FLASH_ERROR;
    }
    else if (flash_id[1] != MX25L3233F_MEMORY_TYPE)
    {
		return FLASH_ERROR;
	}
    else if (flash_id[2] != MX25L3233F_MEMORY_CAPACITY)
	{
        return FLASH_ERROR;
	}
    else
	{
        return FLASH_OK;
	}
}

/**
	Erases the specified FLASH sector.
*/
uint8_t Serial_Flash_EraseSector(EFlash_Handle_t * pHandle, uint32_t SectorAddr)
{
    // Sector Erase 
    MX25L3233F_WriteEnable(&pHandle->eFlash);

    MX25L3233F_BlockErase(&pHandle->eFlash, SectorAddr, MX25L3233F_ERASE_4K);

    // Wait the end of Flash writing and Deselect the FLASH
    if(MX25L3233F_AutoPollingMemReady(&pHandle->eFlash, MX25L3233F_GENERAL_TIME_OUT) != MX25L3233F_OK)
    {
        return FLASH_ERROR;
    }
    else
    {
        return FLASH_OK;
    }
}

/**
	Erases the entire FLASH.
*/
uint8_t Serial_Flash_EraseChip(EFlash_Handle_t * pHandle)
{
  // Sector Erase
  MX25L3233F_WriteEnable(&pHandle->eFlash);
  
  MX25L3233F_ChipErase(&pHandle->eFlash);
  
  // Wait the end of Flash writing and Deselect the FLASH
  if(MX25L3233F_AutoPollingMemReady(&pHandle->eFlash, MX25L3233F_CHIP_ERASE_MAX_TIME) != MX25L3233F_OK)
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
uint8_t Serial_Flash_WriteData(EFlash_Handle_t * pHandle, uint32_t WriteAddr, uint8_t* pData, uint32_t Size)
{
  uint8_t ret = FLASH_OK;
  uint32_t end_addr, current_size, current_addr;

  // Calculation of the size between the write address and the end of the page
  current_size = MX25L3233F_PAGE_SIZE - (WriteAddr % MX25L3233F_PAGE_SIZE);
  
  // Check if the size of the data is less than the remaining place in the page
  if (current_size > Size)
  {
    current_size = Size;
  }
  
  // Initialize the address variables 
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;
  
  // Perform the write page by page
  do
  {
    // Enable write operations
    if(MX25L3233F_WriteEnable(&pHandle->eFlash) != MX25L3233F_OK)
    {
      ret = FLASH_ERROR;
    }
    // Issue page program command
    else if(MX25L3233F_PageProgram(&pHandle->eFlash, pData, current_addr, current_size) != MX25L3233F_OK)
    {
      ret = FLASH_ERROR;
    }
    // Configure automatic polling mode to wait for end of program
    else if(MX25L3233F_AutoPollingMemReady(&pHandle->eFlash, MX25L3233F_GENERAL_TIME_OUT) != MX25L3233F_OK)
    {
      ret = FLASH_ERROR;
    }
    // Update the address and size variables for next page programming
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + MX25L3233F_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25L3233F_PAGE_SIZE;
  } while((current_addr < end_addr) && (ret == FLASH_OK));

  // Return BSP status
  return ret;
}

/**
	Reads a block of data from the FLASH.
*/
uint8_t Serial_Flash_ReadData(EFlash_Handle_t * pHandle, uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize)
{
  if(MX25L3233F_Read(&pHandle->eFlash,pData, uwStartAddress, uwDataSize)!= MX25L3233F_OK)
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
uint8_t Serial_Flash_ReadID (EFlash_Handle_t * pHandle, uint8_t* pData)
{
  if(MX25L3233F_ReadID(&pHandle->eFlash, pData) != MX25L3233F_OK)
  {
    return FLASH_ERROR;
  }
  else
  {
    return FLASH_OK;
  }
}
