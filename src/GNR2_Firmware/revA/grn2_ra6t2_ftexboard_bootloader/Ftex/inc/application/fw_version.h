/**
*  @file fw_version.c
*  @author Bruno Alves
*  @brief Application file used to 
*         read/write/update GNR 
*         firmware version from data, 
*         using the uCAL_DATA_FLASH.
*/

#ifndef FW_VERSION_H_
#define FW_VERSION_H_

/*********************************************
               Includes                       
*********************************************/

#include "uCAL_DATAFLASH.h"
#include "fw_update.h"
#include "fw_storage.h"
#include "sysflash.h"

/*********************************************
                Defines
*********************************************/
//define the lenght on bytes of the GNR serial number.
#define FW_VERSION_LENGTH     4 

//max number of attemps when erase data flash.
#define MAX_ERASE_ATTEMPTS    3

//add pad bytes(zeros) to make the define be a multiple of 4(this is necessary to write correctly in 
//the user data flash). write operation on data flash must be to be multiple of 4.
#define NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN  (((FW_VERSION_LENGTH%4) == (0)) ? (FW_VERSION_LENGTH) : (FW_VERSION_LENGTH + 4 - (FW_VERSION_LENGTH%4)))

#define NUMBER_OF_BYTES_IN_THE_BLOCK    64U

//definition used to control how many
//blocks of the data flash memory are being
//used to hold the user configuration.
//each block has 64 bytes.
#define NUMBER_OF_BLOCKS_USED  1
/*********************************************
          Data Struct Definition
*********************************************/


//struct used to hold all necessary pointers
//and variable to have access to the GNR information
//in the data flash.
typedef struct 
{
    uint8_t Fw_Version[4];
    DataFlash_Handle_t *pDataFlash_Handle;
    
}FirwamreVersionHandle_t;

//========================= EXTERN TYPES ==========================//
extern DataFlash_Handle_t DataFlashHandle;


// ==================== Public function prototypes ========================= //
/**
* @brief Function used to read GNR serial number from the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_2 address.
*        This is the 3 block of the data flash.
* @param DataFlash_Handle_t * pDataFlashHandle pointer to the struct that control
*        data flash init.
* @return void
*/
void Fw_ReadFwVersionDataFlash(DataFlash_Handle_t * pDataFlashHandle);

/**
* @brief Function used to write dfu pack version 
*        in the data flash.
*        GNR information is kept in the FLASH_HP_DF_BLOCK_3 address.
*        This is the 3 block of the data flash.
* @param DataFlash_Handle_t * pDataFlashHandle pointer to the struct that control
*        data flash init.
* @return void
*/
void Fw_WriteFwVersionDataFlash(DataFlash_Handle_t * pDataFlashHandle);

/**
* @brief Function used to read firmware version + dfupack version 
*        from flash memory.
* @param none
* @return void
*/
void Fw_ReadFwVersionFlash(void);

#endif