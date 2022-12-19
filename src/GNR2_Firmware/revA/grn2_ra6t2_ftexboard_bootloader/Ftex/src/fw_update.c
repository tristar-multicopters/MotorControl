/**
  ******************************************************************************
  * @file    fw_update.c
  * @brief   This file contain the firmware update file
             using MCUboot parameter
  ******************************************************************************
*/

// =============================== Includes ================================ //

#include "mx25_driver.h"
#include "external_flash.h"
#include "internal_flash.h"
#include "uCAL_SPI.h"
#include "fw_storage.h"
#include "fw_update.h"

#include "sysflash.h"
#include "bootutil/bootutil.h"
#include "bootutil/image.h"

#include "stdlib.h"
#include "uart_debug.h"

// ================================ Defines =============================== //

/* Size of the SPI array */
#define EXTERNAL_MEMORY_CHUNK_SIZE 128            
/* Array for the external falsh memory */
uint8_t external_flash_data[EXTERNAL_MEMORY_CHUNK_SIZE]; 

// ==================== Private function prototypes ======================== //

/**
    Check the First memory Image
*/
int FW_ReadVersion_FirstImage(const struct flash_area *fap, struct image_header *out_hdr)
{
    int rc = 0;
    /* Image slot 0 is the first app */
    rc = flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &fap);
    if (rc == 0) 
    {
        rc = flash_area_read(fap, 0, out_hdr, 32);
        flash_area_close(fap);
    }

    if (rc != 0) 
    {
        rc = BOOT_EFLASH;
    }
    return rc;
}

/**
    Check the memory Second Image
*/
int FW_ReadVersion_SecondImage(const struct flash_area *fap, struct image_header *out_hdr){
    
    int rc = 0;
    /* Image slot 0 is the first app */
    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &fap);
    if (rc == 0) 
    {
        rc = flash_area_read(fap, 0, out_hdr, 32);
        flash_area_close(fap);
    }

    if (rc != 0) 
    {
        rc = BOOT_EFLASH;
    }
    return rc;
}

// ==================== Public function prototypes ======================== //

/**
    Check and update of the FW
*/
int FW_UpdateProccess()
{
    
    /* Iinitialise the FW Update file */
    if(FW_StorageInit() == EXIT_SUCCESS)
    {
        /* Check if an update is required */
        if(FW_CheckUpdate() == UPDATE_REQUIRED)
        {        
            FW_EraseSecondImage();// Clear the "new app" memory region
            FW_WriteSecondImage();// Write the new app from the external flash into the second memory region
        }
        else
        {
            FW_Storage_FinalizePackRead(); 
        }
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

/**
    Check if an update is required
*/
int FW_CheckUpdate(void)
{
    const struct flash_area *fap;
    struct image_header img_h;
    struct image_header img_h_2;
    
    uint32_t GanPower_fw_version;
    
    FW_ReadVersion_FirstImage(fap, &img_h);     // We must check if the current image is lower version
    FW_ReadVersion_SecondImage(fap, &img_h_2);  // We must check if the back image is lower version or equal (could be one taht has been rejected)
    
    if(FW_Storage_StartPackRead())
    { 
        return VERSION_OK;
    }
    
    GanPower_fw_version = FW_Storage_GetGNR_FWVersion();
    
    MSG_LOG_DBG("Dfu pack ren version : %u.%u.%u\r\n",(uint8_t) GanPower_fw_version,(uint8_t) (GanPower_fw_version >>8),(uint16_t) (GanPower_fw_version >> 16));
    if(img_h.ih_ver.iv_major == 0xff && GanPower_fw_version != 0x0000)
    {   /* If not image in the first slot */
        return UPDATE_REQUIRED;
    }
    /* Must be the same Major version */
    if(img_h.ih_ver.iv_major != (uint8_t) (GanPower_fw_version))            
    {
        return VERSION_OK;
    }
    
    /* can be a newer minor version */
    if((img_h.ih_ver.iv_minor >= (uint8_t) ((GanPower_fw_version >> 8) & 0x0f)) 
        || ((img_h_2.ih_ver.iv_minor >= (uint8_t) ((GanPower_fw_version >> 8) & 0x0f)) && img_h_2.ih_ver.iv_minor != 0xff))
    {    
        return VERSION_OK;
    }
    /* Must be a newer revision version for both slot (otherwise we can have a cycling update of an rejected version)*/
    else if((img_h.ih_ver.iv_revision >= (uint16_t) ((GanPower_fw_version >> 16))) 
            || (img_h_2.ih_ver.iv_revision >= (uint16_t) ((GanPower_fw_version >> 16)) && img_h_2.ih_ver.iv_revision != 0xff))
    {
        return VERSION_OK;
    }    
    return UPDATE_REQUIRED;
}

/**
    Erase second image
*/
int  FW_EraseSecondImage(void)
{
    const struct flash_area *fap;
    int rc;

    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &fap);
    if (rc != 0) 
    {
        return FLASH_ERROR;
    }

    flash_area_erase(fap, 0, flash_area_get_size(fap));
    flash_area_close(fap);
    
    return rc;
}
 
/**
    Write a line of flash data into the internal flash
*/
int FW_Write(uint8_t *pData,uint32_t address,uint32_t size)
{
    const struct flash_area *fap;
    int rc;
    
    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &fap);
    if (rc != 0) 
    {
        return FLASH_ERROR;
    }
    flash_area_write(fap ,address,pData ,size);
    flash_area_close(fap);
    
    return rc;
}


/**
    read the external flash and write chunk of data by chunk of data in the secondary image
*/
int  FW_WriteSecondImage(void)
{
    uint32_t internal_address;
    uint8_t data_read_len;
    internal_address = INTERNAL_ADDRESS_0;
 
    do
    {
        data_read_len = FW_Storage_GetPackChunk(external_flash_data, SPI_TRANSFER_SIZE); 
        FW_Write(external_flash_data,internal_address, data_read_len);
        internal_address += data_read_len;
    }while(data_read_len == SPI_TRANSFER_SIZE);
        
    FW_Storage_FinalizePackRead();
    
    return FLASH_OK;
}