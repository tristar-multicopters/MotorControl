/**
  * @file    spi_master.c
  * @brief   This file contaisn the external flash spi driver.
  */
	
	/* Includes ------------------------------------------------------------------*/
#include "fw_storage_test.h"


/* Variables -----------------------------------------------------------------*/
uint8_t buffer[128] = {0};


/* Public Functions ----------------------------------------------------------*/

/**
	Function used for firmware storage read testing
*/
 void fw_storage_test_read()
 {
	 fw_storage_init();

	fw_storage_start_pack_read();
	fw_storage_get_pack_chunk( buffer, 128 );

	fw_storage_finalize_pack_read();
}
/**
	Function used for firmware storage testing
*/
 void fw_storage_test()
{
	ota_header_t h = {0};
	h.data.ble_fw_size = 0;
	h.data.ren_fw_size = 0;
	h.data.gnr_fw_size = 2;
	
	buffer[0] = 0x12;
	buffer[1] = 0x34;
	buffer[2] = 0x56;
	buffer[3] = 0x78;
	buffer[4] = 0x9A;
	buffer[5] = 0xBC;
	
	fw_storage_init(); // InitFlash
	
	fw_storage_start_pack_write(); // Open littlefs
	
	fw_storage_put_pack_chunk( (uint8_t *) Basic_Bootloader_App, 65692 );
    
	fw_storage_finalize_pack_write();

}