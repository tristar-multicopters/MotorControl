/**
  * @file    dfu.c
  * @brief   This file contaisn device firmware update file.
  */

/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"

#include "mx25_driver.h"
#include "serial_flash.h"
#include "int_flash.h"
#include "flash_spi.h"
#include "dfu.h"
#include "sysflash.h"
#include "fw_storage.h"
#include "test.h"

#include "bootutil/bootutil.h"
#include "bootutil/image.h"


#define EXTERNAL_MEMORY_CHUNK_SIZE 128										// Size of the SPI array (abritrary chunk)
uint8_t external_flash_data[EXTERNAL_MEMORY_CHUNK_SIZE]; // Array for the SPI with the external falsh

/***** Private function declaration ****/
int DFU_read_version_first_image(const struct flash_area *fap, struct image_header *out_hdr);
int DFU_read_version_second_image(const struct flash_area *fap, struct image_header *out_hdr);
/***************************************/
int DFU_proccess(){
	
	// Iinitialise the dfu file
	if(fw_storage_init() == EXIT_SUCCESS){
	
		// Check if an update is required
		if(DFU_check_update() == UPDATE_REQUIRED)
		{		
			DFU_erase_second_image();										// Clear the "new app" memory region
			DFU_write_second_image();										// Write the new app from the external flash into the second memory region
		}
		else
		{
			fw_storage_finalize_pack_read();
		}
		return EXIT_SUCCESS;
		}
	else{
		return EXIT_FAILURE;
	}
}

int DFU_check_update(void)
{
	const struct flash_area *fap;
	struct image_header img_h;
	struct image_header img_h_2;
	
	uint32_t GanPower_fw_version;
	
	DFU_read_version_first_image(fap, &img_h);			// We must check if the current image is lower version
	DFU_read_version_second_image(fap, &img_h_2); // We must check if the back image is lower version or equal (could be one taht has been rejected)
	
	if(fw_storage_start_pack_read()){
		return VERSION_OK;
	}
	
	GanPower_fw_version = fw_storage_get_gnr_fw_version();
	
	//MSG_LOG_DBG("Dfu pack ren version : %u.%u.%u\r\n",(uint8_t) GanPower_fw_version,(uint8_t) (GanPower_fw_version >>8),(uint16_t) (GanPower_fw_version >> 16));
	if(img_h.ih_ver.iv_major == 0xff && GanPower_fw_version != 0x0000){ // If not image in the first slot
			return UPDATE_REQUIRED;
	}
	
	if(img_h.ih_ver.iv_major != (uint8_t) (GanPower_fw_version ))			// Must be the same Major version
	{
		return VERSION_OK;
	}
	
	// can be a newer minor version
	if((img_h.ih_ver.iv_minor >= (uint8_t) ((GanPower_fw_version >> 8) & 0x0f)) 
		|| ((img_h_2.ih_ver.iv_minor >= (uint8_t) ((GanPower_fw_version >> 8) & 0x0f)) && img_h_2.ih_ver.iv_minor != 0xff))
	{	
		return VERSION_OK;
	}
		// Must be a newer revision version for both slot (otherwise we can have a cycling update of an rejected version)	
	else if((img_h.ih_ver.iv_revision >= (uint16_t) ((GanPower_fw_version >> 16))) 
			|| (img_h_2.ih_ver.iv_revision >= (uint16_t) ((GanPower_fw_version >> 16)) && img_h_2.ih_ver.iv_revision != 0xff))
	{
		return VERSION_OK;
	}	
	return UPDATE_REQUIRED;
}


int DFU_read_version_first_image(const struct flash_area *fap, struct image_header *out_hdr){
	
	int rc = 0;
	// image slot 0 is the first app
	rc = flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &fap);
	if (rc == 0) {
			rc = flash_area_read(fap, 0, out_hdr, 32);
			flash_area_close(fap);
	}

	if (rc != 0) {
			rc = BOOT_EFLASH;
	}
	return rc;
}

int DFU_read_version_second_image(const struct flash_area *fap, struct image_header *out_hdr){
	
	int rc = 0;;
	// image slot 0 is the first app
	rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &fap);
	if (rc == 0) {
			rc = flash_area_read(fap, 0, out_hdr, 32);
			flash_area_close(fap);
	}

	if (rc != 0) {
			rc = BOOT_EFLASH;
	}
	return rc;
}
	 /**
  * @brief Erase second image
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
  */
	

int  DFU_erase_second_image(void)
{
		const struct flash_area *fap;
    int rc;

    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &fap);
    if (rc != 0) {
        return FLASH_ERROR;
    }

		flash_area_erase(fap, 0, flash_area_get_size(fap));


    flash_area_close(fap);
    return rc;
	}

	 /**
  * @brief  Write a line of flash data into the internal flash
	* @param	Binary data, address to be write, quantity of data
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
  */
	
int DFU_write(uint8_t *pData,uint32_t address,uint32_t size){
		const struct flash_area *fap;
    int rc;
    rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &fap);
    if (rc != 0) {
        return FLASH_ERROR;
    }
		flash_area_write(fap ,address,pData ,size);
    flash_area_close(fap);
    return rc;
}


	 /**
  * @brief  read the external flash and write chunk of data by chunk of data in the secondary image
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
  */

/* TBD full flow*/

int  DFU_write_second_image(void)
{
	uint32_t internal_address;
	uint8_t data_read_len;
	internal_address = 0x00;
	
	do
	{
		data_read_len = fw_storage_get_pack_chunk(external_flash_data, SPI_TRANSFER_SIZE);
		DFU_write(external_flash_data,internal_address, data_read_len);
		internal_address += data_read_len;
	}while(data_read_len == SPI_TRANSFER_SIZE);
		
	fw_storage_finalize_pack_read();
	
	return FLASH_OK;
}