/**
  ******************************************************************************
  * @file    internal_flash.c
  * @brief   This file contain the Internal flash communication driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "internal_flash.h"
#include "hal_data.h"
#include "sysflash/sysflash.h"

// ==================== Public function prototypes ======================== //

/*
    ERASE secondary image
*/
int InternalFlash_EraseSecondaryImage(void)
{
    const struct flash_area *fap;
    int rc;
    /* Internal flash open area */
    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(1), &fap);
    if (rc != 0) 
    {
        return BOOT_EFLASH;
    }
    /* Internal flash erase area */
    flash_area_erase(fap, 0, flash_area_get_size(fap));

    /* Internal flash close area */
    flash_area_close(fap);
    
    return rc;
}
	 
/*
	* Write data in secondary image
*/	
int InternalFlash_Write(uint8_t *pData,uint32_t address,uint32_t size)
{
	const struct flash_area *fap;
    int rc;
    /* Internal flash open area */
    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(1), &fap);
    if (rc != 0) 
    {
        return BOOT_EFLASH;
    }
    /* Internal flash erase area */
    flash_area_write(fap ,address,pData ,size);
    /* Internal flash close area */
    flash_area_close(fap);
    
    return rc;
}