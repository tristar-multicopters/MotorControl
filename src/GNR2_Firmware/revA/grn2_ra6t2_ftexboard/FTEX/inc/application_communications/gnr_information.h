/**
* @file   grn_information.h
* @author Bruno Alves
* @brief  Application file used to read GNR information,
*         as serial number, firmware version and etc.
*         
*
*/

#ifndef GNR_INFORMATION_H_
#define GNR_INFORMATION_H_

/*********************************************
               Includes                       
*********************************************/

#include "uCAL_DATAFLASH.h"
#include "comm_config.h"


/*********************************************
                Defines
*********************************************/
//define the length of bytes of the GNR serial number.
#define GNR_INFO_SERIAL_LENGTH     12
//define the length off bytes in the GNR serial Header "EPC"
#define GNR_INFO_SERIAL_HEADER_LENGTH     3 


#define GNR_INFO_ODOMETER_ADDR      FLASH_HP_DF_BLOCK_49
//define the lenght of bytes of the odomoeter value
//Lenght is 5 because 4 bytes for the value and 1 byte for crc (3 empty bytes)
#define GNR_INFO_ODOMETER_LENGTH    8

//define the length of bytes of the dfu pack version.
#define GNR_DFUPACK_VERSION_LENGTH  4 

/******External FLash address to store the serial **********************/

#define EXT_FLASH_OTP_SERIAL_ADDRESS MX25L3233F_OTP_START_ADDRESS


/*********************************************
          Data Struct Definition
*********************************************/

//struct used to hold all necessary pointers
//and variable to have access to the GNR information
//in the data flash.
typedef struct 
{
    uint8_t  Gnr_DfuPackVersion[GNR_DFUPACK_VERSION_LENGTH];
    uint8_t  Gnr_serialNumber[GNR_INFO_SERIAL_LENGTH]; 
    uint8_t  Gnr_Odometer[GNR_INFO_ODOMETER_LENGTH];
    
    DataFlash_Handle_t * pDataFlash_Handle;
    EFlash_Storage_Handle_t * pEFlashStorageHandle;
}GnrInfoHandle_t;

//========================= EXTERN TYPES ==========================//



// ==================== Public function prototypes ========================= //
/**
* @brief Function used to read GNR serial number from the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_2 address.
*        This is the 3 block of the data flash.
* @param DataFlash_Handle_t * pDataFlashHandle pointer to the struct that control
*        data flash init.
* @return void
*/
void GnrInfo_Init(DataFlash_Handle_t * pDataFlashHandle, 
                  EFlash_Storage_Handle_t * pEFlashStorageHandle);
/**
* @brief Function used to read GNR serial number from the data flash.
* @param none
* @return uint64_t GNR serial number.
*/
uint64_t GnrInfo_GetSerialNumber(void);

/**
* @brief Function used to read GNR dfu pack version from the data flash. 
* @param none
* @return uint32_t GNR dfu pack version.
*/
uint32_t  GnrInfo_GetDFuPackVersion(void);

uint32_t GnrInfo_GetOdometer(void);

void GnrInfo_DownloadOdometer(uint32_t new_odometer);

#endif