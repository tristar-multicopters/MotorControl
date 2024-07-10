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

#define IMAGE_SIZE_MCU_512K 0x70000

/************** CRC32 Macros ******************/
//
#define FLASH_READ_DWORD(x) (*(uint32_t*)(x))

/*********************************************
          Data Struct Definition
*********************************************/

//constante table used to calculate the crc32.
static const uint32_t crc32_table[] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

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

/**
    function used to calculate the firmware crc32 and compare with the crc32
    write at the end of the firmware file. if ok return true.
*/
bool FW_CheckFirmwareIntegrity(void);

/**********************************************************************************************************************
 * @brief Reset crcState variable to 0xFFFFFFFF
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Reset(void) ;

/**********************************************************************************************************************
 * @brief Calculate CRC-32 using polynomial x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x1 + 1
 * @param const uint8_t data Data over which the crc is computed
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Update(const uint8_t data);

/**********************************************************************************************************************
 * @brief Return crc 32 value.
 * @return  uint32_t return the crc32 calculated value.
 **********************************************************************************************************************/
uint32_t FirmwareUpdate_Crc32Result(void);

/**********************************************************************************************************************
 * @brief Return firmVersionDetected flag.
 * @return  none.
 **********************************************************************************************************************/
bool FW_FirmVersionDetected(void);


#endif /* __FW_UPDATE_H */