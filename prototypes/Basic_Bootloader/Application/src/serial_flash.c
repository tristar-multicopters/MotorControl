/**
  * @file    serial_flash.c
  * @brief   This file contaisn device serial flash driver.
  */

/* Includes ------------------------------------------------------------------*/
#include "serial_flash.h"

/* Functions ------------------------------------------------------------------*/

/**
  * @brief  Initializes peripherals used by the Serial FLASH device.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t BSP_SERIAL_FLASH_Init(void)
{
  if(MX25L3233F_ResetEnable() != MX25L3233F_OK) return FLASH_ERROR;

  R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

  if(MX25L3233F_ResetMemory() != MX25L3233F_OK) return FLASH_ERROR;

 R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
	
	uint8_t flash_id[3] = { 0 };
	BSP_SERIAL_FLASH_ReadID(flash_id);
	
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
  * @brief  Erases the specified FLASH sector.
  * @param  SectorAddr: address of the sector to erase.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t BSP_SERIAL_FLASH_EraseSector(uint32_t SectorAddr)
{
  /*!< Sector Erase */
  MX25L3233F_WriteEnable();

  MX25L3233F_BlockErase(SectorAddr, MX25L3233F_ERASE_4K);

  /*!< Wait the end of Flash writing and Deselect the FLASH*/
  if(MX25L3233F_AutoPollingMemReady(MX25L3233F_GENERAL_TIME_OUT) != MX25L3233F_OK)
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
uint8_t BSP_SERIAL_FLASH_EraseChip(void)
{
  /*!< Sector Erase */
  MX25L3233F_WriteEnable();
  
  MX25L3233F_ChipErase();
  
  /*!< Wait the end of Flash writing and Deselect the FLASH*/
  if(MX25L3233F_AutoPollingMemReady(MX25L3233F_CHIP_ERASE_MAX_TIME) != MX25L3233F_OK)
  {
    return FLASH_ERROR;
  }
  else
  {
    return FLASH_OK;
  }
}

/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pData: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  uwStartAddress: FLASH's internal address to write to.
  * @param  uwDataSize: number of bytes to write to the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t BSP_SERIAL_FLASH_WriteData(uint32_t WriteAddr, uint8_t* pData, uint32_t Size)
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
    if(MX25L3233F_WriteEnable() != MX25L3233F_OK)
    {
      ret = FLASH_ERROR;
    }/* Issue page program command */
    else if(MX25L3233F_PageProgram(pData, current_addr, current_size) != MX25L3233F_OK)
    {
      ret = FLASH_ERROR;
    }/* Configure automatic polling mode to wait for end of program */
    else if(MX25L3233F_AutoPollingMemReady(MX25L3233F_GENERAL_TIME_OUT) != MX25L3233F_OK)
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
  * @brief  Reads a block of data from the FLASH.
  * @param  pData: pointer to the buffer that receives the data read from the FLASH.
  * @param  uwStartAddress: FLASH's internal address to read from.
  * @param  uwDataSize: number of bytes to read from the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t BSP_SERIAL_FLASH_ReadData(uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize)
{
  if(MX25L3233F_Read(pData, uwStartAddress, uwDataSize)!= MX25L3233F_OK)
  {
    return FLASH_ERROR;
  }
  else
  {
    return FLASH_OK;
  }
}

/**
  * @brief  Reads FLASH identification.
  * @retval FLASH identification
  */
uint8_t BSP_SERIAL_FLASH_ReadID(uint8_t* pData)
{
  if(MX25L3233F_ReadID(pData) != MX25L3233F_OK)
  {
    return FLASH_ERROR;
  }
  else
  {
    return FLASH_OK;
  }
}
