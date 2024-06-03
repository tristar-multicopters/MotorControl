/**
*  @file gnr_information.c
*  @brief Application file used to read GNR information,
*         as serial number, firmware version and etc, 
*         using the uCAL_DATA_FLASH.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "gnr_information.h"
#include "ASSERT_FTEX.h"

/*********************************************
                Defines
*********************************************/

/*********************************************
                Private Variables
********************************************/

//struct used to hold all necessary pointers
//and variable to have access to the GNR information
//in the data flash.
static GnrInfoHandle_t GnrInfoHandle;

/*********************************************
                Public Variables
*********************************************/

// ==================== Private Functions ======================== //

/**
    Read the serial from the external flash
*/

static uint8_t ExtFLash_ReadSerial(EFlash_Storage_Handle_t * pEFlashStorageHandle,
                                   uint8_t * data,
                                   uint32_t adress, 
                                   uint32_t size);

/**
    Write the serial to the external flash
*/

static uint8_t ExtFLash_WriteSerial(EFlash_Storage_Handle_t * pEFlashStorageHandle,
                                   uint8_t * data,
                                   uint32_t adress, 
                                   uint32_t size);
/**
    Verify if the serial exist
    The clear value for a flash memory is 0xFF
*/

static bool SerialExist(uint8_t * pSerial);

// ==================== Public Functions ======================== //// ==================== Public Functions ======================== //
/**
* @brief Function used to read GNR serial number from the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_2 and 3.
*        This is the 3 block of the data flash.
*/
void GnrInfo_Init(DataFlash_Handle_t * pDataFlashHandle, 
                  EFlash_Storage_Handle_t * pEFlashStorageHandle)
{
    //verify the pointers.
    ASSERT(pDataFlashHandle != NULL);
    ASSERT(pEFlashStorageHandle != NULL);
    //pass the pointer that control data flash
    GnrInfoHandle.pDataFlash_Handle = pDataFlashHandle;
    
    //pass the pointer that control External flash
    GnrInfoHandle.pEFlashStorageHandle = pEFlashStorageHandle;
    //try to open flash memory
    if(uCAL_Data_Flash_Open(GnrInfoHandle.pDataFlash_Handle) == true)
    {
        //get GNR serial number from data flash memory, on the block FLASH_HP_DF_BLOCK_2.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle, GnrInfoHandle.Gnr_serialNumber, FLASH_HP_DF_BLOCK_2, GNR_INFO_SERIAL_LENGTH);
            
        //get GNR dfu pack version from data flash memory, on the block FLASH_HP_DF_BLOCK_3.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle, GnrInfoHandle.Gnr_DfuPackVersion, FLASH_HP_DF_BLOCK_3, GNR_DFUPACK_VERSION_LENGTH);
    }
    //Close data flash memory access.
    uCAL_Data_Flash_Close(GnrInfoHandle.pDataFlash_Handle);
    
    //*****OPT verification
    bool serial_exist = SerialExist(&GnrInfoHandle.Gnr_serialNumber[0]);
    
    uint8_t OTP_Serial[GNR_INFO_SERIAL_LENGTH] = {0};
    bool OTP_serial_exist;
    ExtFLash_ReadSerial(GnrInfoHandle.pEFlashStorageHandle,
                        OTP_Serial,
                        EXT_FLASH_OTP_SERIAL_ADDRESS,
                        GNR_INFO_SERIAL_LENGTH);
    
    OTP_serial_exist = SerialExist(&OTP_Serial[0]);
    // Check if the serial exists in the data flash
    if (serial_exist) 
    {
        // Check if it does not exist in the OTP
        if (!OTP_serial_exist)
        {
            // Serial is in the data flash but not in the OTP, write it.
            ExtFLash_WriteSerial(GnrInfoHandle.pEFlashStorageHandle,
                                 GnrInfoHandle.Gnr_serialNumber,
                                 EXT_FLASH_OTP_SERIAL_ADDRESS,
                                 GNR_INFO_SERIAL_LENGTH);
        }
    }
    else
    {
        // Check if it exists in the OTP
        if (OTP_serial_exist)
        {
            // Serial is not in the data flash but is in the OTP, update GnrInfoHandle.Gnr_serialNumber
            for (uint8_t index = 0; index < GNR_INFO_SERIAL_LENGTH; index++) 
            {
                GnrInfoHandle.Gnr_serialNumber[index] = OTP_Serial[index];
            }
        }
        else 
        {
            // In the odd case that neither ext flash nor int data flash, clear.
            for (uint8_t index = 0; index < GNR_INFO_SERIAL_LENGTH; index++) 
            {
                GnrInfoHandle.Gnr_serialNumber[index] = 0;
            }
        }
    }
}

/**
* @brief Function used to read GNR serial number from the data flash. 
*        Only the first 8 bytes of the serial number are provide to
*        from this function.
*/
uint64_t  GnrInfo_GetSerialNumber(void)
{
    uint64_t serialNumber = 0;
    uint8_t m = 0;
    
    //copy the serial number byte by byte to the variable.
    //starting from the years byte until the last by of the batch number.
    for(uint8_t n = 1; n <= GNR_INFO_SERIAL_LENGTH - 4; n++)
    {
        serialNumber = serialNumber | ((uint64_t)GnrInfoHandle.Gnr_serialNumber[GNR_INFO_SERIAL_LENGTH - n] << m*8);
        
        //increment to the next shift.
        m++;
    }
    
    return serialNumber;   
}

/**
* @brief Function used to read GNR dfu pack version from the data flash.  
*/
uint32_t  GnrInfo_GetDFuPackVersion(void)
{
    uint32_t dfuPackVersion = 0;
    uint8_t m = 0;
    
    //copy dfu pack version byte by byte to the variable.
    for(short int n = 3; n >= 0; n--)
    {
        //move byte by byte from the array the variable.
        dfuPackVersion = dfuPackVersion | ((uint32_t)GnrInfoHandle.Gnr_DfuPackVersion[n] << (m*8));
        
        //increment to the next shift.
        m++;
    }
    
    return dfuPackVersion;   
}

//=================Private functions ===========================//

static uint8_t ExtFLash_WriteSerial(EFlash_Storage_Handle_t * pEFlashStorageHandle,
                                   uint8_t * data,
                                   uint32_t address, 
                                   uint32_t size)
{
    return Serial_Flash_OTP_WriteData(&pEFlashStorageHandle->eFlashStorage, data, address, size);
}


static uint8_t ExtFLash_ReadSerial(EFlash_Storage_Handle_t * pEFlashStorageHandle,
                                   uint8_t * data,
                                   uint32_t address, 
                                   uint32_t size)
{
    return Serial_Flash_OTP_ReadData(&pEFlashStorageHandle->eFlashStorage, data, address, size);
}


static bool SerialExist(uint8_t * pSerial)
{
    ASSERT(pSerial != NULL);
    ASSERT(GNR_INFO_SERIAL_LENGTH > GNR_INFO_SERIAL_HEADER_LENGTH);
    
    //The serial number includes the acronym for:
    // (E) Evionics, (P) Power, (C) Controller
    // Flash Erase do not clear to 0xFF in the data flash
    bool ret = false;
    if(pSerial[0] == 'E' && pSerial[1] == 'P' && pSerial[2] == 'C')
    {
        ret = true;
    }    
    return ret;
}
