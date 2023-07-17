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
//define the lenght on bytes of the GNR serial number.
#define GNR_INFO_SERIAL_LENGTH     13 

/*********************************************
          Data Struct Definition
*********************************************/

//struct used to hold all necessary pointers
//and variable to have access to the GNR information
//in the data flash.
typedef struct 
{
    uint8_t  Gnr_serialNumber[13];
    DataFlash_Handle_t *pDataFlash_Handle;
    
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
void GnrInfo_Read(DataFlash_Handle_t * pDataFlashHandle);

/**
* @brief Function used to read GNR serial number from the data flash.
* @param none
* @return uint64_t GNR serial number.
*/
uint64_t GnrInfo_GetSerialNumber(void);

#endif