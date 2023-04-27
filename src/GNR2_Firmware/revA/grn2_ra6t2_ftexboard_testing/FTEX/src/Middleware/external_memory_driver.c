/**
  ******************************************************************************
  * @file    external memory driver.c
  * @brief   This file contaisn the driver for the external flash memory.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "external_memory_driver.h"

// ========================== Public Variables ============================ //

timer_status_t g_timer0_status;

// ==================== Public function prototypes ======================== //
/**
	Polling Write In Progress bit become to 0 Get Flash information
*/
int32_t SF_AutoPollingMemReady(uint32_t Timeout)
{
    int ret = MX25L3233F_OK;

    /* Select the Serial Flash: Chip Select low */
    uCAL_SPI_Enable();
  
    /* Send read ID instruction */
    uCAL_SPI_WriteByte(MX25L3233F_READ_STATUS_REG_CMD);

	g_timer0.p_api->open(g_timer0.p_ctrl,g_timer0.p_cfg);
	g_timer0.p_api->periodSet(g_timer0.p_ctrl,Timeout);
	g_timer0.p_api->start(g_timer0.p_ctrl);
	g_timer0.p_api->statusGet(g_timer0.p_ctrl,&g_timer0_status);
  
    while(g_timer0_status.state == TIMER_STATE_COUNTING)
    {
		g_timer0.p_api->statusGet(g_timer0.p_ctrl,&g_timer0_status);
		if((uCAL_SPI_WriteByte(CLOCK_FOR_READ) & MX25L3233F_SR_WIP) == 0){
			goto MX25L3233F_AutoPollingMemReady_OK;
		}
    }
    ret = MX25L3233F_ERROR_AUTOPOLLING;
	MX25L3233F_AutoPollingMemReady_OK :
	
    /* Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
	
    g_timer0.p_api->close(g_timer0.p_ctrl);
  
    return ret;
}

/**
    Function to Full Ease the external flash memory.
*/
int32_t SF_ChipErase(void)
{
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Write Enable" instruction */
    /* Read/Write Array Commands (3/4 Byte Address Command Set) */
    uCAL_SPI_WriteByte(MX25L3233F_CHIP_ERASE_CMD);
	
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
    return MX25L3233F_OK;
}

/**
    Reads an amount of data from the SPI memory on STR mode.
*/
int32_t SF_Read(uint8_t *pData, uint32_t Address, uint32_t Size)
{
  if(Size == 0) return MX25L3233F_ERROR_PARAMETER;
 
  /* Select the Serial Flash Chip Select low */
  uCAL_SPI_Enable();
 
  /* Send Write Enable instruction */
  uCAL_SPI_WriteByte(MX25L3233F_READ_CMD);
 
  /* Send SectorAddr high address byte */
  uCAL_SPI_WriteByte((Address >> 16) & 0xFF);
  /* Send SectorAddr medium address byte */
  uCAL_SPI_WriteByte((Address >> 8) & 0xFF);
  /* Send SectorAddr low address byte */
  uCAL_SPI_WriteByte(Address & 0xFF);
 
  /* Dummy clock if use Fast read command */
  for(uint32_t i = 0; i < Size; i++) *(pData++) = uCAL_SPI_WriteByte(CLOCK_FOR_READ);
 
  /* Select the Serial Flash Chip Select high */
  uCAL_SPI_Disable();
 
  return MX25L3233F_OK;
}

/**
	Writes an amount of data to the QSPI memory
*/
int32_t SF_PageProgram(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    if (Size == 0) 
        return MX25L3233F_ERROR_PARAMETER;
 
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Write Enable" instruction */
    uCAL_SPI_WriteByte(MX25L3233F_PAGE_PROG_CMD);

    /*!< Send SectorAddr high address byte */
    uCAL_SPI_WriteByte((Address >> 16) & 0xFF);
    /*!< Send SectorAddr medium address byte */
    uCAL_SPI_WriteByte((Address >> 8) & 0xFF);
    /*!< Send SectorAddr low address byte */
    uCAL_SPI_WriteByte(Address & 0xFF);

    for(uint32_t i = 0; i < Size; i++) uCAL_SPI_WriteByte(*(pData++));
	
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();

    return MX25L3233F_OK;
}

/**
    Erases the specified block of the QSPI memory.
    MX25L3233F support 4K, 32K, 64K size block erase commands.
*/
int32_t SF_BlockErase(uint32_t BlockAddress, MX25L3233F_EraseTypeDef BlockSize)
{
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send Sector Erase instruction */
    switch(BlockSize)
    {
    case MX25L3233F_ERASE_4K :
        uCAL_SPI_WriteByte(MX25L3233F_SECTOR_ERASE_4K_CMD);
        break;

    case MX25L3233F_ERASE_32K :
        uCAL_SPI_WriteByte(MX25L3233F_BLOCK_ERASE_32K_CMD);
        break;

    case MX25L3233F_ERASE_64K :
        uCAL_SPI_WriteByte(MX25L3233F_BLOCK_ERASE_64K_CMD);
        break;

    case MX25L3233F_ERASE_CHIP :
        return SF_ChipErase();

    default :
        return MX25L3233F_ERROR_PARAMETER;
    }

    /*!< Send SectorAddr high address byte */
    uCAL_SPI_WriteByte((BlockAddress >> 16) & 0xFF);
    /*!< Send SectorAddr medium address byte */
    uCAL_SPI_WriteByte((BlockAddress >> 8) & 0xFF);
    /*!< Send SectorAddr low address byte */
    uCAL_SPI_WriteByte(BlockAddress & 0xFF);
	
    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();

    return MX25L3233F_OK;
}

/**
    Write Register/Setting Commands 
*/
int32_t SF_WriteEnable(void)
{  	
    /*!< Select the FLASH: Chip Select low */
    uCAL_SPI_Enable();
    /*!< Send "Write Enable" instruction */
    uCAL_SPI_WriteByte(MX25L3233F_WRITE_ENABLE_CMD);

    /*!< Deselect the FLASH: Chip Select high */
    uCAL_SPI_Disable();
    return MX25L3233F_OK;
}
/**
    Read Flash 3 Byte IDs.
*/
int32_t SF_ReadID(uint8_t *ID)
{
  /* Select the Serial Flash Chip Select low */
  uCAL_SPI_Enable();
 
  /* Send Read ID instruction */
  uCAL_SPI_WriteByte(MX25L3233F_READ_ID_CMD);
  
  /* Read a byte from the Serial Flash */
  ID[0] = uCAL_SPI_WriteByte(CLOCK_FOR_READ);
  
  /* Read a byte from the Serial Flash */
  ID[1] = uCAL_SPI_WriteByte(CLOCK_FOR_READ);
  
  /* Read a byte from the Serial Flash */
  ID[2] = uCAL_SPI_WriteByte(CLOCK_FOR_READ);
  
  /* Deselect the FLASH: Chip Select high */
  uCAL_SPI_Disable();

  return MX25L3233F_OK;
}

/**
    Flash Reset
*/
int32_t SF_ResetEnable(void)
{
  /* Select the Serial Flash Chip Select low */
  uCAL_SPI_Enable();

  /* Send Reset Enable instruction */
  uCAL_SPI_WriteByte(MX25L3233F_RESET_ENABLE_CMD);

  /* Select theSerial Flash Chip Select high */
  uCAL_SPI_Disable();
  return MX25L3233F_OK;
}

/**
    Flash Reset memory
*/
int32_t SF_ResetMemory(void)
{
  /* Select the Serial Flash  Chip Select low */
  uCAL_SPI_Enable();

  /* Send Reset Enable instruction */
  uCAL_SPI_WriteByte(MX25L3233F_RESET_MEMORY_CMD);

  /* Select the Serial Flash  Chip Select high */
  uCAL_SPI_Disable();
  return MX25L3233F_OK;
}
