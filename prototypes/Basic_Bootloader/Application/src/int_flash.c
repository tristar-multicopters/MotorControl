/**
  * @file    int_flash.c
  * @brief   This file contaisn the internal flash driver.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "int_flash.h"
#include "hal_data.h"
#include "sysflash/sysflash.h"

/* Public Functions ----------------------------------------------------------*/
 
/**
  * @brief  Erases the internal secondary image.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
  */
 
int INTERNAL_FLASH_erase_secondary_image(void)
{
		const struct flash_area *fap;
    int rc;

    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(1), &fap);
    if (rc != 0) {
        return BOOT_EFLASH;
    }

		flash_area_erase(fap, 0, flash_area_get_size(fap));


    flash_area_close(fap);
    return rc;
}
	 
/**
  * @brief  Write a line of flash data into the internal flash
	* @param	hexline with all the information size/adress and data
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
  */
	
int INTERNAL_FLASH_write(uint8_t *pData,uint32_t address,uint32_t size)
{
		const struct flash_area *fap;
    int rc;
    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(1), &fap);
    if (rc != 0) {
        return BOOT_EFLASH;
    }
		flash_area_write(fap ,address,pData ,size);
    flash_area_close(fap);
    return rc;
}