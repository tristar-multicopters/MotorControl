/**
  ******************************************************************************
  * @file    internal_memory.c
  * @brief   This file contain the internal data flash memory bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //

#include "internal_memory.h"

volatile bool dataFlashOpenFlag;
flash_result_t blank_check_result;
// ==================== Public function prototypes ======================== //

/**
  * @brief  Function to open the flash memory access.
  * @param  None
  * @return bool return true if operation was a success.
  */
bool Internal_Memory_Open()
{
    //Open the flash hp instance. */
    fsp_err_t err = R_FLASH_HP_Open(&g_flash0_ctrl,&g_flash0_cfg);

    //if success ok
    if(FSP_SUCCESS == err)
    {
        //set the open flag to indicate memory access is open.
        dataFlashOpenFlag = true;
        return true;
    }
	return false; 
}

/**
  * @brief  Function to close the flash memory access.
  * @param  None
  * @return bool return true if operation was a success.
  */
bool Internal_Memory_Close()
{
    //Close the flash hp instance. */
    fsp_err_t err = R_FLASH_HP_Close(&g_flash0_ctrl);
    //if success ok
    if(FSP_SUCCESS == err)
    {
        //clear the open flag to indicate memory access is closed.
        dataFlashOpenFlag = false;
        return true;
    }
	return false;  
}

/**
  * @brief  Function to erase the data flash memory.
  * @param  uint32_t const blockAddress start block address to be erased. 
  * @oaram  uint32_t const numBlocks number of black to be erased.
  * @return bool return true if the data memory was correctly reased and false if not
            or a wrong parameter was passed.
*/
bool Internal_Memory_Erase(uint32_t const blockAddress, uint32_t const numBlocks)
{	

    //Verify if the block address is inside of the data flash memory space and if the
    //number of block to be erased is greater than zero.
    if((blockAddress >= DATA_FLASH_START_ADDRESS) && (blockAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBlocks > 0))
    {
        //Verify if the total number of block to be erase are outside of the data flash.
        if(blockAddress + (numBlocks - 1)*DATA_FLASH_BLOCK_OFFSET_64BYTES <= DATA_FLASH_LAST_BLOCK_ADDRESS)
        {
            //Open the flash hp instance. */
            fsp_err_t err = R_FLASH_HP_Erase(&g_flash0_ctrl,blockAddress, numBlocks);
            //verify if the blank check commmand was sent and if the memory was
            //correctly erased.
            if(FSP_SUCCESS == err)
            {
                return true;
            }
        }
    }

	return false;
}

/**
  * @brief  Function to write bytes in the data flash memory.
  * @param  uint8_t * data pointer to the data to be written in the data flash memory.
  * @param  uint32_t const flashAddress start address to be written. 
  * @param  uint32_t const numBytes number of bytes to be written.
  * @return bool return true if the data was correctly written to memory and false if not
  or a wrong parameter was passed.
*/
bool Internal_Memory_Write(uint8_t * data, uint32_t flashAddress, uint32_t const numBytes)
{      
    //Verify if the flash address is inside of the data flash memory space and if the
    //number of bytes to be written is greater than zero.
    if((flashAddress >= DATA_FLASH_START_ADDRESS) && (flashAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBytes > 0))
    {
        
        //Verify if the total number of bytes to be written are outside of the data flash.
        if(flashAddress + (numBytes - 1) <= DATA_FLASH_END_ADDRESS)
        {
            
            //Write numBytes in the data flash memory. */
            fsp_err_t err = R_FLASH_HP_Write(&g_flash0_ctrl,(uint32_t)data, flashAddress, numBytes);
            
            //verify if the blank check commmand was sent and if the memory was
            //correctly erased.
            if(FSP_SUCCESS == err)
            {
                return true;   
            }
        }
    }
  
    return false;
}
	
/**
  * @brief  Function to read bytes in the data flash memory.
            To read bytes from data flash memory, data flash access
            must to be on open state.
  * @param  uint8_t * data pointer to the buffer that will receive data 
            from the data flash memory.
  * @param  uint32_t const flashAddress start address to be read. 
  * @param  uint32_t const numBytes number of bytes to be read.
  * @return bool true if the data was read and false if not.
*/
bool Internal_Memory_Read(uint8_t * data, uint32_t flashAddress, uint32_t const numBytes)
{

    //Verify if the flash address is inside of the data flash memory space and if the
    //number of bytes to be read is greater than zero.
    if((flashAddress >= DATA_FLASH_START_ADDRESS) && (flashAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBytes > 0))
    {
        //Verify if the total number of bytes to be read are outside of the data flash.
        if(flashAddress + (numBytes - 1) <= DATA_FLASH_END_ADDRESS)
        {
            //copy bytes from the flashAddress to the buffer.
            memcpy(data, (uint8_t *)flashAddress, numBytes);
            return true;
        }
    }
    return false;
}
	
/**
  * @brief  Function to performe a blank check in the data flash memory.
            Blanck check is used to verify if the data flash contents was correctly 
            erased(0xFF). Read data from the data flash give a randon value each time 
            you make the read, if the region was erased but no programed(write operation).
            So, the only way to verify if memory was erased if calling the blank check 
            function.
  * @param  FLASH_Handle_t that allow to configure all necessary parameters.
  * @param  uint32_t const blockAddress start block address to be blank checked. 
  * @oaram  uint32_t const numBytes number of bytes to be blank checked(at least 4 bytes).
  * @return bool return true if the memory was correctly erased and false if not
  or a wrong parameter was passed.
*/
bool Internal_Memory_Blank_Check(uint32_t const blockAddress, uint32_t numBytes)
{      
    //Verify if the block address is inside of the data flash memory space and if the
    //number of block to be blank checked are greater than four.
    if((blockAddress >= DATA_FLASH_START_ADDRESS) && (blockAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBytes > 4))
    {
        
        //Verify if the total number of block to be blank checked are outside of the data flash.
        if(blockAddress + (numBytes - 1) <= DATA_FLASH_LAST_BLOCK_ADDRESS)
        {
            
            //Blanck check the data flash memory.
            fsp_err_t err = R_FLASH_HP_BlankCheck(&g_flash0_ctrl, blockAddress, numBytes, &blank_check_result);
            
            //verify if the blank check commmand was sent and if the memory was
            //correctly erased.
            if(FSP_SUCCESS == err)
            {
                return true;
            }
        }
    }
    
    return false;
}
