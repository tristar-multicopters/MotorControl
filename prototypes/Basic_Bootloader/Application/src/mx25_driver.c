/**
  * @file    mx25_driver.c
  * @brief   This file contaisn the driver for the external flash memory.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "mx25_driver.h"

/* Private define ------------------------------------------------------------*/

timer_status_t g_timer0_status;

/**
	Polling Write In Progress bit become to 0 Get Flash information
*/
int32_t MX25L3233F_AutoPollingMemReady(uint32_t Timeout)
{
  int ret = MX25L3233F_OK;

  /*!< Select the FLASH: Chip Select low */
  FLASH_SPI_CS_LOW();
  
  /*!< Send "RDID " instruction */
  FLASH_SPI_IO_WriteByte(MX25L3233F_READ_STATUS_REG_CMD);

	g_timer0.p_api->open(g_timer0.p_ctrl,g_timer0.p_cfg);
	g_timer0.p_api->periodSet(g_timer0.p_ctrl,Timeout);
	g_timer0.p_api->start(g_timer0.p_ctrl);
	g_timer0.p_api->statusGet(g_timer0.p_ctrl,&g_timer0_status);
  
  while(g_timer0_status.state == TIMER_STATE_COUNTING)
  {
		g_timer0.p_api->statusGet(g_timer0.p_ctrl,&g_timer0_status);
		if((FLASH_SPI_IO_WriteByte(CLOCK_FOR_READ) & MX25L3233F_SR_WIP) == 0){
			goto MX25L3233F_AutoPollingMemReady_OK;
		}
  }
  
  ret = MX25L3233F_ERROR_AUTOPOLLING;
  
	MX25L3233F_AutoPollingMemReady_OK :
	
  /*!< Deselect the FLASH: Chip Select high */
  FLASH_SPI_CS_HIGH();
	g_timer0.p_api->close(g_timer0.p_ctrl);
  
	return ret;
}

/**
    Reads an amount of data from the SPI memory on STR mode.
*/
int32_t MX25L3233F_Read(uint8_t *pData, uint32_t Address, uint32_t Size)
{
  if(Size == 0) return MX25L3233F_ERROR_PARAMETER;
 
  /*!< Select the FLASH: Chip Select low */
  FLASH_SPI_CS_LOW();
 
  /*!< Send "Write Enable" instruction */
  FLASH_SPI_IO_WriteByte(MX25L3233F_READ_CMD);
 
  /*!< Send SectorAddr high address byte */
  FLASH_SPI_IO_WriteByte((Address >> 16) & 0xFF);
  /*!< Send SectorAddr medium address byte */
  FLASH_SPI_IO_WriteByte((Address >> 8) & 0xFF);
  /*!< Send SectorAddr low address byte */
  FLASH_SPI_IO_WriteByte(Address & 0xFF);
 
  /*
  Dummy clock if use Fast read command
  */
  
  for(uint32_t i = 0; i < Size; i++) *(pData++) = FLASH_SPI_IO_WriteByte(CLOCK_FOR_READ);
 
  /*!< Select the FLASH: Chip Select high */
  FLASH_SPI_CS_HIGH();
 
  return MX25L3233F_OK;
}

/**
    Read Flash 3 Byte IDs.
*/
int32_t MX25L3233F_ReadID(uint8_t *ID)
{
  /*!< Select the FLASH: Chip Select low */
  FLASH_SPI_CS_LOW();
  
  /*!< Send "RDID " instruction */
  FLASH_SPI_IO_WriteByte(MX25L3233F_READ_ID_CMD);
  

  /*!< Read a byte from the FLASH */
  ID[0] = FLASH_SPI_IO_WriteByte(CLOCK_FOR_READ);
  
  /*!< Read a byte from the FLASH */
  ID[1] = FLASH_SPI_IO_WriteByte(CLOCK_FOR_READ);
  
  /*!< Read a byte from the FLASH */
  ID[2] = FLASH_SPI_IO_WriteByte(CLOCK_FOR_READ);
  
  /*!< Deselect the FLASH: Chip Select high */
  FLASH_SPI_CS_HIGH();

  return MX25L3233F_OK;
}

/**
    Flash Reset
*/
int32_t MX25L3233F_ResetEnable(void)
{
  /*!< Select the FLASH: Chip Select low */
  FLASH_SPI_CS_LOW();

  /*!< Send "Reset Enable" instruction */
  FLASH_SPI_IO_WriteByte(MX25L3233F_RESET_ENABLE_CMD);

  /*!< Select the FLASH: Chip Select high */
  FLASH_SPI_CS_HIGH();
  return MX25L3233F_OK;
}

/**
    Flash Reset memory
*/
int32_t MX25L3233F_ResetMemory(void)
{
  /*!< Select the FLASH: Chip Select low */
  FLASH_SPI_CS_LOW();

  /*!< Send "Reset Enable" instruction */
  FLASH_SPI_IO_WriteByte(MX25L3233F_RESET_MEMORY_CMD);

  /*!< Select the FLASH: Chip Select high */
  FLASH_SPI_CS_HIGH();
  return MX25L3233F_OK;
}
