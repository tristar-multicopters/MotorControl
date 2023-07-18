/**
*  @file fw_version.c
*  @author Bruno Alves
*  @brief Application file used to 
*         read/write/update GNR 
*         firmware version from data, 
*         using the uCAL_DATA_FLASH.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "fw_version.h"
#include "ASSERT_FTEX.h"

/*********************************************
                Defines
*********************************************/

/*********************************************
        Private Function Declaration
*********************************************/

//Read the First memory Image and get the image header information.
static int Fw_ReadVersionFirstImage(const struct flash_area *fap, struct image_header *out_hdr);

/*********************************************
                Private Variables
********************************************/

//struct used to hold all necessary pointers
//and variable to have access to the GNR information
//in the data flash.
static FirwamreVersionHandle_t FirwamreVersionHandle;

/*********************************************
                Public Variables
*********************************************/
//Handle to control the data flash initialisation 
DataFlash_Handle_t DataFlashHandle =
{
	
	.pFlashInstance = &g_flash0,
	.dataFlashOpenFlag = false,

};

// ==================== Public Functions ======================== //
/**
* @brief Function used to read dfu pack version 
*        from the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_3 address.
*        This is the 3 block of the data flash.
*/
void Fw_ReadFwVersionDataFlash(DataFlash_Handle_t * pDataFlashHandle)
{
    //verify the pointers.
    ASSERT(pDataFlashHandle != NULL);
    
    //pass the pointer that control data flash
    FirwamreVersionHandle.pDataFlash_Handle = pDataFlashHandle;
    
    //try to open flash memory
    if(uCAL_Flash_Open(FirwamreVersionHandle.pDataFlash_Handle) == true)
    {   
        //read GNR dfupack version from data flash memory, on the block FLASH_HP_DF_BLOCK_3.
        uCAL_Data_Flash_Read(FirwamreVersionHandle.pDataFlash_Handle, FirwamreVersionHandle.Fw_Version, FLASH_HP_DF_BLOCK_3, FW_VERSION_LENGTH);   
    }
    
    //Close data flash memory access.
    uCAL_Flash_Close(FirwamreVersionHandle.pDataFlash_Handle);
}

/**
* @brief Function used to write dfu pack version 
*        in the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_3 address.
*        This is the 3 block of the data flash.
*/
void Fw_WriteFwVersionDataFlash(DataFlash_Handle_t * pDataFlashHandle)
{
    //verify the pointers.
    ASSERT(pDataFlashHandle != NULL);
    
    //variable used to control how my attempts will be done
    //when trying to erase data flash memory.
    uint8_t attempts;
    
    //pass the pointer that control data flash
    FirwamreVersionHandle.pDataFlash_Handle = pDataFlashHandle;
    
    //try to open flash memory
    if(uCAL_Flash_Open(FirwamreVersionHandle.pDataFlash_Handle) == true)
    {      
        //try max three times to erase data flash memory.
        for(attempts = 0; attempts < MAX_ERASE_ATTEMPTS ; attempts++)
        {
            //erase one block(block 0) of the data flash memory before write on it.
            //if you need more than 64 to write user configuration we need to
            //erase more than 1 block.
            uCAL_Data_Flash_Erase(FirwamreVersionHandle.pDataFlash_Handle, FLASH_HP_DF_BLOCK_3, NUMBER_OF_BLOCKS_USED);
            
            //check if all bytes were erased.
            //we are using just 1 block, so just 64 are checked.
            //to check 2 or more blocks use n*NUMBER_OF_BYTES_IN_THE_BLOCK.
            if (uCAL_Data_Flash_Blank_Check(FirwamreVersionHandle.pDataFlash_Handle, FLASH_HP_DF_BLOCK_3, NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
            { 
                //if memory was correctly erased break
                //stop the attempts.
                break;               
            }
        }
                
        //write user configuration that represents USER_DATA_CONFIG_LENGTH bytes.
        uCAL_Data_Flash_Write(FirwamreVersionHandle.pDataFlash_Handle,FirwamreVersionHandle.Fw_Version,FLASH_HP_DF_BLOCK_3, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN);
    }
    
    //Close data flash memory access.
    uCAL_Flash_Close(FirwamreVersionHandle.pDataFlash_Handle);
}

// ==================== Public Functions ======================== //
/**
* @brief Function used to read firmware version + dfupack version 
*        from flash memory.
*        
*/
void Fw_ReadFwVersionFlash(void)
{
    //
    const struct flash_area *fap;
    struct image_header img_h;
    
    //struct used to hold firmwares 
    //version.
    version_t dfu_pack;
    version_t version_img_1;
    
    //read header info about the first image slot.
    Fw_ReadVersionFirstImage(fap, &img_h);
    
    //copy dfu pack version into the version_t version_img_1.
    version_img_1.vers_concat = img_h.ih_ver.iv_build_num;
    
    //read dfu pack version from data flash.
    Fw_ReadFwVersionDataFlash(&DataFlashHandle);
    
    //verify if dfu pack firmware version inside of flash memory is valid.
    if((version_img_1.hardware != 0x00) && (version_img_1.hardware != 0xFF) && (version_img_1.bikeModel != 0xFF) && (version_img_1.revision != 0xFFFF))
    {
        //copy the dfu pack version, read from data flash, to the dfu_pack struct.
        dfu_pack.hardware = FirwamreVersionHandle.Fw_Version[0];
        dfu_pack.bikeModel = FirwamreVersionHandle.Fw_Version[1];
        dfu_pack.revision = (uint16_t)((FirwamreVersionHandle.Fw_Version[2] << 8) | FirwamreVersionHandle.Fw_Version[3]);
        
        //compare firmware data flash and flash version
        //if they are different is necessary to update data flash
        //firmware version.
        if ((dfu_pack.hardware != version_img_1.hardware) || (dfu_pack.bikeModel != version_img_1.bikeModel) || (dfu_pack.revision != version_img_1.revision))
        {
            //copy the new dfu pack version to the array
            FirwamreVersionHandle.Fw_Version[0] = version_img_1.hardware;
            FirwamreVersionHandle.Fw_Version[1] = version_img_1.bikeModel;
            FirwamreVersionHandle.Fw_Version[2] = (uint8_t)(version_img_1.revision >> 8);
            FirwamreVersionHandle.Fw_Version[3] = (uint8_t)version_img_1.revision;
            
            //write the new dfu pack version in the data flash memory.
            Fw_WriteFwVersionDataFlash(&DataFlashHandle);
        }
    }
}

// ==================== Private Functions ======================== //

/**
* @brief Read the First memory Image and get the image header information.
* @param const struct flash_area *fap Structure describing an area on a flash device.
* @param struct image_header *out_hdr
*/
static int Fw_ReadVersionFirstImage(const struct flash_area *fap, struct image_header *out_hdr)
{
    int rc = 0;
    /* Image slot 0 is the first app */
    rc = flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &fap);
    if (rc == 0) 
    {
        rc = flash_area_read(fap, 0, out_hdr, 32);
        flash_area_close(fap);
    }
    
    //if rc is not zero, error happned when reading
    //internal flash memory.
    if (rc != 0) 
    {
        rc = BOOT_EFLASH;
    }
    
    return rc;
}
