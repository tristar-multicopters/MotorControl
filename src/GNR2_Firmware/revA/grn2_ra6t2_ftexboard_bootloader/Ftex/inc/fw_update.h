/**
  ******************************************************************************
  * @file    fw_update.h
  * @brief   This file is the fw update header
  ******************************************************************************
*/

#ifndef __FW_UPDATE_H
#define __FW_UPDATE_H

// =============================== Includes ================================ //
#include "bsp_api.h"

// ================================ Defines =============================== //
#define VERSION_OK          0x00    // Firmware version is ok and no need for firmware update
#define UPDATE_REQUIRED     0x01    // Firmware Update is required
#define UNKNOWN_ERROR       0xFF    // Error Got from Boot

#define INTERNAL_ADDRESS_0  0x00;   // Internal Flash memory start adress

// ==================== Public function prototypes ======================== //

/**
  * @brief  Check and update of the FW
  * @param  None
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
int FW_UpdateProccess();

/**
  * @brief  check if an update is required
  * @param  None
  * @retval FW_OK (0x00) if no update required, else 
  *         return UPDATE_TO_DO (0x01).
  */
int FW_CheckUpdate(void);

/**
  * @brief  Erase the first image
  * @param  None
  * @retval FLASH_OK (0x00) if no update required, else 
  *         return FLASH_ERROR (0x01).
  */
int  FW_EraseFirstImage(void);
	
/**
  * @brief  Update the second image with the external flash
  * @param  None
  * @retval FLASH_OK (0x00) if no update required, else 
  *         return FLASH_ERROR (0x01).
  */
int  FW_WriteFirstImage(void);


#endif /* __FW_UPDATE_H */