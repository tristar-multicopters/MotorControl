/**
  ******************************************************************************
  * @file    serial_flash.c
  * @author  FTEX inc
  * @brief   Serial Flash Module handle the firmware pack/file manipulation via an
  *          external SPI flash memory
  ******************************************************************************
*/
// =============================== Includes ================================= //
#include "serial_flash_storage.h"

// =============================== Privates Definition ================================= //



/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/



/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/

// ==================== Public function prototypes ======================== //

/**
    Initializes the serial flash storage
*/
uint8_t SF_Storage_Init(EFlash_Storage_Handle_t * pHandle)
{
	
	if(Serial_Flash_Init(&pHandle->eFlashStorage) != FLASH_OK)
	{
		return EXIT_FAILURE;
	}
	else
	{	
		return FLASH_OK;
	}	
}
