/**
  ******************************************************************************
  * @file    internal_flash_storage.c
  * @author  FTEX inc
  * @brief   Internal Flash storage handle the serial number manipulation
  ******************************************************************************
*/
// =============================== Includes ================================= //
#include "internal_flash_storage.h"

// =============================== Defines ================================= //


static uint8_t  ReadData[20]= {0};

//define the lenght on bytes of the User_ConfigData_t.
#define USER_DATA_CONFIG_LENGTH   12
    
//add pad bytes(zeros) to make the define be a multiple of 4(this is necessary to write correctly in 
//the user data flash). write operation on data flash must be to be multiple of 4.
#define NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN  (((USER_DATA_CONFIG_LENGTH%4) == (0)) ? (USER_DATA_CONFIG_LENGTH) : (USER_DATA_CONFIG_LENGTH + 4 - (USER_DATA_CONFIG_LENGTH%4)))


// ==================== Public function prototypes ======================== //

/**
  * @brief Function to write user data config into the data flash memory.
  * @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
           to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  * @return void
*/
void InternalMemory_WriteSerialNumber(uint8_t SaveData[SERIAL_LENGTH])
{
    /* variable used to control how my attempts will be done when trying to erase data flash memory.*/
    uint8_t bAttempts;
      
    //try to open flash memory
    if (Internal_Memory_Open() == true)
    {  
        //try max three times to erase data flash memory.
        for(bAttempts = 0; bAttempts < MAX_ERASE_ATTEMPTS ; bAttempts++)
        {  
            // Erase one block(block 2) of the data flash memory before write on it.
            Internal_Memory_Erase(FLASH_HP_DF_BLOCK_2, NUMBER_OF_BLOCKS_USED);
        
            // Check if all bytes were erased.
            if (Internal_Memory_Blank_Check(FLASH_HP_DF_BLOCK_2, NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
            {           
                // If memory was correctly erased break stop the attempts.
                break; 
            }
        }
      
        // Check if the memory was erased in less than three attempts
        if (bAttempts < MAX_ERASE_ATTEMPTS )
        {
            // Write user configuration that represents USER_DATA_CONFIG_LENGTH bytes.
            Internal_Memory_Write(SaveData,FLASH_HP_DF_BLOCK_2, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN);
        }
    }
    //Close data flash memory access.
    Internal_Memory_Close();    
}


/***/
void InternalMemory_ReadSerialNumber(void)
{
     if (Internal_Memory_Open() == true)
    { 
        // Read user configuration that represents USER_DATA_CONFIG_LENGTH bytes.
        Internal_Memory_Read(ReadData,FLASH_HP_DF_BLOCK_2, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN); 
    }    
    UART_SerialPrint((uint8_t *)"SerialNumber: ");
    UART_SerialPrint(ReadData);
    UART_SerialPrint((uint8_t *)"  ");
    //Close data flash memory access.
    Internal_Memory_Close();
}    