/**
  * @file    dfu.h
  * @brief   This file is the device firmware update header
	*/

#ifndef __DFU_H
#define __DFU_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_api.h"

/* Defines --------------------------------------------------------------------*/
#define VERSION_OK 			0x00
#define UPDATE_REQUIRED 0x01
#define UNKNOWN_ERROR		0xff

/* Function Prototypes ------------------------------------------------------------*/

/**
  * @brief  Check and update of the FW
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
int DFU_proccess(void);

/**
  * @brief  check if an update is required
  * @retval FW_OK (0x00) if no update required, else 
  *         return UPDATE_TO_DO (0x01).
  */
int DFU_check_update(void);
/**
  * @brief  Erase the second image
 */
int DFU_erase_second_image(void);

/**
  * @brief  Update the second image with the external flash
  * @retval FLASH_OK (0x00) if no update required, else 
  *         return FLASH_ERROR (0x01).
  */
int DFU_write_second_image(void);



#endif /* __DFU_H */