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


// ==================== Public Functions ======================== //
/**
* @brief Function used to read GNR serial number from the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_2 and 3.
*        This is the 3 block of the data flash.
*/
void GnrInfo_Read(DataFlash_Handle_t * pDataFlashHandle)
{
    //verify the pointers.
    ASSERT(pDataFlashHandle != NULL);
    
    //pass the pointer that control data flash
    GnrInfoHandle.pDataFlash_Handle = pDataFlashHandle;
    
    //try to open flash memory
    if(uCAL_Flash_Open(GnrInfoHandle.pDataFlash_Handle) == true)
    {   
        //get GNR serial number from data flash memory, on the block FLASH_HP_DF_BLOCK_2.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle, GnrInfoHandle.Gnr_serialNumber, FLASH_HP_DF_BLOCK_2, GNR_INFO_SERIAL_LENGTH);
        
        //get GNR dfu pac version from data flash memory, on the block FLASH_HP_DF_BLOCK_3.
        uCAL_Data_Flash_Read(GnrInfoHandle.pDataFlash_Handle, GnrInfoHandle.Gnr_DfuPackVersion, FLASH_HP_DF_BLOCK_3, GNR_DFUPACK_VERSION_LENGTH);
    }
    
    //Close data flash memory access.
    uCAL_Flash_Close(GnrInfoHandle.pDataFlash_Handle);
}

/**
* @brief Function used to read GNR serial number from the data flash. 
*        Only the first 8 bytes of the serial number are provide to
*        from this function.
*/
uint64_t GnrInfo_GetSerialNumber(void)
{
    uint64_t serialNumber = 0;
    // Hardcoding "23490001" in ASCII directly into a uint64_t variable
    serialNumber |= (uint64_t)'2' << 56; // Most significant byte (MSB)
    serialNumber |= (uint64_t)'3' << 48;
    serialNumber |= (uint64_t)'3' << 40;
    serialNumber |= (uint64_t)'7' << 32;
    serialNumber |= (uint64_t)'0' << 24;
    serialNumber |= (uint64_t)'0' << 16;
    serialNumber |= (uint64_t)'9' << 8;
    serialNumber |= (uint64_t)'9'; // Least significant byte (LSB)
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
