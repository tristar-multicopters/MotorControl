/**
  ******************************************************************************
  * @file    MX25l3233F_driver.c
  * @author  FTEX inc
  * @brief   MX25L3233F driver module to manage the external Flash Memory
  ******************************************************************************
*/

// =============================== Includes ================================= //
#include "MX25L3233F_driver.h"
// =============================== Defines ================================= //
#define CLOCK_FOR_READ	0x55	/* For generate read clock */

// ==================== Public function prototypes ======================== //

/**
    Get External Flash information 
    Sector/Page/Block size details and values
*/
int32_t MX25L3233F_GetFlashInfo(MX25L3233F_Info_t *pInfo)
{
    /* Configure the structure with the memory configuration */
    pInfo->FlashSize          = MX25L3233F_FLASH_SIZE;
    pInfo->EraseSectorSize    = MX25L3233F_SECTOR_4K;
    pInfo->EraseSectorsNumber = (MX25L3233F_FLASH_SIZE/MX25L3233F_SECTOR_4K);
    pInfo->ProgPageSize       = MX25L3233F_PAGE_SIZE;
    pInfo->ProgPagesNumber    = (MX25L3233F_FLASH_SIZE/MX25L3233F_PAGE_SIZE);

    pInfo->EraseBlockSize     = MX25L3233F_BLOCK_32K;
    pInfo->EraseBlockNumber   = (MX25L3233F_FLASH_SIZE/MX25L3233F_BLOCK_32K);
    pInfo->EraseBlockSize1    = MX25L3233F_BLOCK_64K;
    pInfo->EraseBlockNumber1  = (MX25L3233F_FLASH_SIZE/MX25L3233F_BLOCK_64K);
    return MX25L3233F_OK;
}

/**
	Polling Write In Progress bit to Get Flash information
    A memory status will be returned
*/
int32_t MX25L3233F_AutoPollingMemReady(MX25_Handle_t *pHandle, uint32_t Timeout)
{
    uint32_t Time;
    int ret = MX25L3233F_OK;
	
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
	
    /*!< Send "RDID " instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_READ_STATUS_REG_CMD);

    //initialize timecount.
    Time = 0;
  
    while(Time < Timeout)
    {
        //enable wdt kick to avoid wdt reset
        #if FIRMWARE_RELEASE
        //must be enabled before production
        Watchdog_Refresh();
        #endif
        
        if((uCAL_SPI_IO_WriteByte(pHandle->uCALSPI,CLOCK_FOR_READ) & MX25L3233F_SR_WIP) == 0) goto MX25L3233F_AutoPollingMemReady_OK;
        
        //add a delay of 1000us to timeout count
        R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MICROSECONDS);
        
        //increment timeout counter.
        Time++;
    }
  
    ret = MX25L3233F_ERROR_AUTOPOLLING;
  
    MX25L3233F_AutoPollingMemReady_OK :

    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
	
    return ret;
}

/**
    Function to Full Ease the external flash memory.
*/
int32_t MX25L3233F_ChipErase(MX25_Handle_t *pHandle)
{
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Write Enable" instruction */
    /* Read/Write Array Commands (3/4 Byte Address Command Set) */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_CHIP_ERASE_CMD);
	
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
    return MX25L3233F_OK;
}

/**
    Reads an amount of data from the SPI Memory on STR mode
*/
int32_t MX25L3233F_Read(MX25_Handle_t *pHandle, uint8_t *pData, uint32_t Address, uint32_t Size)
{	
    if (Size == 0) 
        return MX25L3233F_ERROR_PARAMETER;
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
	
    /* Read/Write Array Commands (4 Byte Address Command Set) */
    /*!< Send "Write Enable" instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_READ_CMD);
 
    /*!< Send SectorAddr high address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, (Address >> 16) & 0xFF);
    /*!< Send SectorAddr medium address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, (Address >> 8) & 0xFF);
    /*!< Send SectorAddr low address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, Address & 0xFF);
 
    /* Dummy clock if use Fast read command */
    for(uint32_t i = 0; i < Size; i++) *(pData++) = uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, CLOCK_FOR_READ);

    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
	
    return MX25L3233F_OK;
}

/**
	Writes an amount of data to the QSPI memory
*/
int32_t MX25L3233F_PageProgram(MX25_Handle_t *pHandle, uint8_t *pData, uint32_t Address, uint32_t Size)
{
    if (Size == 0) 
        return MX25L3233F_ERROR_PARAMETER;
 
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Write Enable" instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_PAGE_PROG_CMD);

    /*!< Send SectorAddr high address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, (Address >> 16) & 0xFF);
    /*!< Send SectorAddr medium address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, (Address >> 8) & 0xFF);
    /*!< Send SectorAddr low address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, Address & 0xFF);

    for(uint32_t i = 0; i < Size; i++) uCAL_SPI_IO_WriteByte(pHandle->uCALSPI,*(pData++));
	
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();

    return MX25L3233F_OK;
}

/**
    Erases the specified block of the QSPI memory.
    MX25L3233F support 4K, 32K, 64K size block erase commands.
*/
int32_t MX25L3233F_BlockErase(MX25_Handle_t *pHandle, uint32_t BlockAddress, MX25L3233F_EraseTypeDef BlockSize)
{
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send Sector Erase instruction */
    switch(BlockSize)
    {
    case MX25L3233F_ERASE_4K :
        uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_SECTOR_ERASE_4K_CMD);
        break;

    case MX25L3233F_ERASE_32K :
        uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_BLOCK_ERASE_32K_CMD);
        break;

    case MX25L3233F_ERASE_64K :
        uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_BLOCK_ERASE_64K_CMD);
        break;

    case MX25L3233F_ERASE_CHIP :
        return MX25L3233F_ChipErase(pHandle);

    default :
        return MX25L3233F_ERROR_PARAMETER;
    }

    /*!< Send SectorAddr high address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, (BlockAddress >> 16) & 0xFF);
    /*!< Send SectorAddr medium address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, (BlockAddress >> 8) & 0xFF);
    /*!< Send SectorAddr low address byte */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, BlockAddress & 0xFF);
	
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();

    return MX25L3233F_OK;
}

/**
    Write Register/Setting Commands 
*/
int32_t MX25L3233F_WriteEnable(MX25_Handle_t *pHandle)
{  	
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Write Enable" instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_WRITE_ENABLE_CMD);

    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
    return MX25L3233F_OK;
}

/**
    Read Flash 3 Byte IDs.
*/
int32_t MX25L3233F_ReadID(MX25_Handle_t *pHandle, uint8_t *ID)
{ 
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "RDID " instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_READ_ID_CMD);
  
    /*!< Read a byte from the FLASH */
    ID[0] = uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, CLOCK_FOR_READ);
  
    /*!< Read a byte from the FLASH */
    ID[1] = uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, CLOCK_FOR_READ);
  
    /*!< Read a byte from the FLASH */
    ID[2] = uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, CLOCK_FOR_READ);
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
    return MX25L3233F_OK;
}

/**
    External SPI Flash Reset
*/
int32_t MX25L3233F_ResetEnable(MX25_Handle_t *pHandle)
{
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Reset Enable" instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_RESET_ENABLE_CMD);
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();	
    return MX25L3233F_OK;
}
/**
    External SPI Flash Reset memory
*/
int32_t MX25L3233F_ResetMemory(MX25_Handle_t *pHandle)
{
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Reset Enable" instruction */
    uCAL_SPI_IO_WriteByte(pHandle->uCALSPI, MX25L3233F_RESET_MEMORY_CMD);
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
    return MX25L3233F_OK;
}
