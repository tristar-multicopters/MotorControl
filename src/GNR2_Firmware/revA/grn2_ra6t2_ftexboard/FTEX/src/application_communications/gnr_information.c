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
        //get GNR serial number from data flash memory, in the block FLASH_HP_DF_BLOCK_2.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle, GnrInfoHandle.Gnr_serialNumber, FLASH_HP_DF_BLOCK_2, GNR_INFO_SERIAL_LENGTH);
                
        //get GNR dfu pack version from data flash memory, on the block FLASH_HP_DF_BLOCK_3.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle, GnrInfoHandle.Gnr_DfuPackVersion, FLASH_HP_DF_BLOCK_3, GNR_DFUPACK_VERSION_LENGTH);
        
        //get GNR odometer from data flash memory, in the block FLASH_HP_DF_BLOCK_49.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle,GnrInfoHandle.Gnr_Odometer,GNR_INFO_ODOMETER_ADDR ,GNR_INFO_ODOMETER_LENGTH);
    }
    
    //Close data flash memory access.
    uCAL_Data_Flash_Close(GnrInfoHandle.pDataFlash_Handle);
    
    //*****OPT verification
    bool serial_exist = false;
    if(GnrInfoHandle.Gnr_serialNumber[0] == 'E' && 
       GnrInfoHandle.Gnr_serialNumber[1] == 'P' && 
       GnrInfoHandle.Gnr_serialNumber[2] == 'C')
    {
        serial_exist = true;
    }
    
    uint8_t OTP_Serial[GNR_INFO_SERIAL_LENGTH] = {0};
    ExtFLash_ReadSerial(GnrInfoHandle.pEFlashStorageHandle, &OTP_Serial[0], EXT_FLASH_OTP_SERIAL_ADDRESS, GNR_INFO_SERIAL_LENGTH);
    bool OTP_serial_exist = false;
    //OTP_serial_exist = SerialExist(&OTP_Serial[0]);
    if(OTP_Serial[0] == 'E' && OTP_Serial[1] == 'P' && OTP_Serial[2] == 'C')
    {
        OTP_serial_exist = true;
    }
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

/**
* @brief Function used to read GNR odometer the data flash. 
*        CRC is also checked to make sure we have a valid odometer value
*/
uint32_t GnrInfo_GetOdometer(void)
{
    uint32_t odometer = 0;
    uint8_t CRC = 0;
    
    CRC = GnrInfoHandle.Gnr_Odometer[0] + GnrInfoHandle.Gnr_Odometer[1] + GnrInfoHandle.Gnr_Odometer[2] + GnrInfoHandle.Gnr_Odometer[3];  
    
    
    if (CRC == GnrInfoHandle.Gnr_Odometer[4])
    {      
        odometer += (uint32_t)(GnrInfoHandle.Gnr_Odometer[0]);
        odometer += (uint32_t)(GnrInfoHandle.Gnr_Odometer[1] << 8);
        odometer += (uint32_t)(GnrInfoHandle.Gnr_Odometer[2] << 16);
        odometer += (uint32_t)(GnrInfoHandle.Gnr_Odometer[3] << 24);
    }
    
    return odometer;   
}

    
    uint8_t OpenResult;
    uint8_t EraseResult;
    uint8_t WriteResult;
    uint8_t CloseResult;
/**
* @brief Function used to write GNR odometer on the data flash. 
*        CRC is also checked to make sure we have a valid odometer value
*/
void GnrInfo_DownloadOdometer(uint32_t new_odometer)
{
    uint8_t odometer[GNR_INFO_ODOMETER_LENGTH];
    // Split the uint32 in 4 bytes
    odometer[0] = (uint8_t) (new_odometer & 0x000000FF);
    odometer[1] = (uint8_t)((new_odometer & 0x0000FF00) >> 8);
    odometer[2] = (uint8_t)((new_odometer & 0x00FF0000) >> 16);
    odometer[3] = (uint8_t)((new_odometer & 0xFF000000) >> 24);
    
    // Compute the CRC
    odometer[4] = odometer[0] + odometer[1] + odometer[2] + odometer[3];
    
    odometer[5] = 0;
    odometer[6] = 0;
    odometer[7] = 0;
    
    OpenResult = uCAL_Data_Flash_Open(GnrInfoHandle.pDataFlash_Handle);
    
    if (OpenResult)
    {
        EraseResult = uCAL_Data_Flash_Erase(GnrInfoHandle.pDataFlash_Handle,FLASH_HP_DF_BLOCK_49,1);
        if (EraseResult)
        {             
            WriteResult = uCAL_Data_Flash_Write(GnrInfoHandle.pDataFlash_Handle,odometer,GNR_INFO_ODOMETER_ADDR,GNR_INFO_ODOMETER_LENGTH);
        }
        
        CloseResult = uCAL_Data_Flash_Close(GnrInfoHandle.pDataFlash_Handle);        
    }    
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
