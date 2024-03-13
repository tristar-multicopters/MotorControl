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
#include "watchdog.h"

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

//variable used to hold the crc value calculated on the fly(streaming).
static uint32_t crcState = (uint32_t)~0L; 

//variable used to identify if signature, with a correct version, was detected
//in the firmware.
static bool firmVersionDetected = false;

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
            //kick to reset WDT timeout.
            Watchdog_Refresh();
            
            //check firmware Integrity.
            if (FW_CheckFirmwareIntegrity())
            {
                //kick to reset WDT timeout.
                Watchdog_Refresh();
                
                // Clear the "new app" memory region
                FW_EraseFirstImage();
                
                //kick to reset WDT timeout.
                Watchdog_Refresh();
                
                //moves file postion to beginning of the GNR
                //firmware to prepare the system to
                //copy GNR firmware from external memory
                //into microcontroller's memory.
                FW_Storage_SetFileToGnrFirmw();
                
                // Write the new app from the external flash into the first memory region
                FW_WriteFirstImage();
            }
            else
            {
                //close file.
                FW_Storage_FinalizePackRead();
            }
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
    
    //struct used to hold firmwares 
    //version.
    version_t dfu_pack;
    version_t version_img_1;
    
    FW_ReadVersion_FirstImage(fap, &img_h);     // We must check if the current image is lower version
    
    //try to open file to get dfu pack 
    //information. If file can't be openned
    //bypass checkupdate.
    if(FW_Storage_StartPackRead())
    {
        return VERSION_OK;
    }
    
    // WE USE THE BUILD_REVISION TO STORE THE DFU PACK
    version_img_1.vers_concat = img_h.ih_ver.iv_build_num;
	dfu_pack.vers_concat = FW_Storage_GetPack_FWVersion();
	
    //Verify if there is a firmware in the microcontroller flash memory.
    //if is true, set the firmVersionDetected.
    //this flag is necessary to be used to detect a corrupted firmware flashed in the
    //microcontroller.
    //Major, minor and version are checked.
    if((version_img_1.hardware != 0x00) && (version_img_1.hardware != 0xFF) && (version_img_1.bikeModel != 0xFF) && (version_img_1.revision != 0xFFFF))			
    {
        //one firmware was detected in the microcontroller flash memory.
        firmVersionDetected = true;
    }
    
    //if the code arrive here is because the external memory has
    //a dfu file.
    //check if dfu pack version of the firmware == 0xfffffff, mcu flash memory is empty.
    //check if dfu pack version of dfu pack file is empty or corroupted(0x00000000).
    if(version_img_1.vers_concat == 0xffffffff && dfu_pack.vers_concat != 0x00000000 && dfu_pack.vers_concat != 0xFFFFFFFF)
    {
        return UPDATE_REQUIRED;
    }
	
    // Must be the same Major version(hardware version), otherwise launch the current one
    if(version_img_1.hardware != dfu_pack.hardware)
    {
        return UPDATE_REQUIRED;
    }
    
    // if not the same bike model don't update and keep the current one
    if(version_img_1.bikeModel != dfu_pack.bikeModel)
    {	
        return UPDATE_REQUIRED;
    }
    else if(version_img_1.revision >= dfu_pack.revision)
    {
        // If not a newere version don't upgrade
        return VERSION_OK;
    }	
    else
    {
        return UPDATE_REQUIRED;
    }
}

/**
    Erase first image
*/
int  FW_EraseFirstImage(void)
{
    const struct flash_area *fap;
    int rc;

    rc = flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &fap);
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
    
    rc = flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &fap);
    if (rc != 0) 
    {
        return FLASH_ERROR;
    }
    flash_area_write(fap ,address,pData ,size);
    flash_area_close(fap);
    
    return rc;
}


/**
    read the external flash and write chunk of data by chunk of data in the first image
*/
int  FW_WriteFirstImage(void)
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

/**
    function used to calculate the firmware crc32 and compare with the crc32
    write at the end of the firmware file. if ok return true.
*/
bool FW_CheckFirmwareIntegrity(void)
{
    uint8_t data_read_len;

    //variable used to calculate crc from the read bytes.
    uint8_t n = 0;
    
    //Variable used to receive the calculated crc.
    uint32_t crc32 = 0;
    
    //set file to the beginning.
    FW_Storage_SetFileToBeginning();
    
    //reset crc calcule
    FirmwareUpdate_Crc32Reset();
    
    //read the whole file.
    do
    {
        //read 16 bytes, #define SPI_TRANSFER_SIZE 16.
        data_read_len = FW_Storage_ReadBytesFromFile(external_flash_data, SPI_TRANSFER_SIZE); 
        
        //calculate the crc for the data_read_len bytes read from the file.
        for(n = 0; n < data_read_len; n++)
        {
            //calculate crc 4 on the fly.
            FirmwareUpdate_Crc32Update(external_flash_data[n]);
        }
    }while(data_read_len == SPI_TRANSFER_SIZE);
    
    //read the last four bytes of the file. Theses bytes
    //represent CRC32 calculated by the GNR when writing
    //in the external memory.
    data_read_len = FW_Storage_ReadBytes(external_flash_data, 4);
    
    //Copy crc 4 bytes from buffer to crc32.
    memcpy(&crc32,&external_flash_data[0],sizeof(crc32));
        
    //
    if (FirmwareUpdate_Crc32Result() == crc32)
    {
       return true; 
    }
    else
    {
       return false; 
    }   
}


/**********************************************************************************************************************
 * @brief Reset crcState variable to 0xFFFFFFFF
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Reset(void) 
{
    crcState = (uint32_t)~0L;
}

/**********************************************************************************************************************
 * @brief Calculate CRC-32 using polynomial x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x1 + 1
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Update(const uint8_t data) 
{
    //
    uint8_t tableIndex = 0;
    
    //Calculate crc32 on the fly.
    tableIndex = (uint8_t)(crcState ^ (data >> (0 * 4)));
    crcState = FLASH_READ_DWORD(crc32_table + (tableIndex & 0x0f)) ^ (crcState >> 4);
    tableIndex = (uint8_t)(crcState ^ (data >> (1 * 4)));
    crcState = FLASH_READ_DWORD(crc32_table + (tableIndex & 0x0f)) ^ (crcState >> 4);
}

/**********************************************************************************************************************
 * @brief Return crc 32 value.
 **********************************************************************************************************************/
uint32_t FirmwareUpdate_Crc32Result(void) 
{
    return ~crcState;
}

/**********************************************************************************************************************
 * @brief Return firmVersionDetected flag.
 **********************************************************************************************************************/
bool FW_FirmVersionDetected(void)
{
    //
    return firmVersionDetected;
}