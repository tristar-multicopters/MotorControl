/**
*  uCal_DataFlash.c
*  uController Abstraction Layer for FLASH
*/ 

/*********************************************
               Includes                       
*********************************************/

#include "uCal_DataFlash.h"
#include "ASSERT_FTEX.h"

/*********************************************
                Defines
*********************************************/

//timeout 
#define FLASH_TIMEOUT    100000000

/*********************************************
                Private Variables
*********************************************/

//variable used to hold values from data flash
//interruption.
static uint8_t flashEvent = 0xFF;

/*********************************************
                Public Variables
*********************************************/




/******************************************************************
                    Private functions Prototype                          
*******************************************************************/

/**
  @brief Function to check interruption response releated to ISR coming
  from the data flash interruption(erased flash, wrote flash, etc).
  This function hold the code until interruption event happens.
  
  @param uint32_t timeout to be used as precaution if the flash
         interruption doesn't happened.
  @return uint8_t returns flashEvent.
*/
static uint8_t uCAL_Is_Flash_Busy(uint32_t timeOut);


/******************************************************************
                    Public functions                             
*******************************************************************/

/**
  @brief Function to open the flash memory access.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @return bool return true if operation was a success.
*/
bool uCAL_Flash_Open(DataFlash_Handle_t * pHandle)
{
	
	//verify if the pointer is not null. 
	ASSERT(pHandle != NULL);
	
	//Verify if the flash memory access was opnned.
	if(pHandle->dataFlashOpenFlag == false)
	{ 
	
		//Open the flash hp instance. */
		fsp_err_t err = R_FLASH_HP_Open(pHandle->pFlashInstance->p_ctrl,pHandle->pFlashInstance->p_cfg);
  
		//if success ok
		if(FSP_SUCCESS == err)
		{
	
			//set the open flag to indicate memory access is open.
			pHandle->dataFlashOpenFlag = true;
			
			//
			return true;
			
		}
		
	}
	
	//
	return false;
    
}

/**
  @brief Function to close the flash memory access.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @return bool return true if operation was a success.
*/
bool uCAL_Flash_Close(DataFlash_Handle_t * pHandle)
{
	
	//verify if the pointer is not null. 
	ASSERT(pHandle != NULL);
	
	//Verify if the flash memory access was opnned.
	if(pHandle->dataFlashOpenFlag == true)
	{
	
		//Close the flash hp instance. */
		fsp_err_t err = R_FLASH_HP_Close(pHandle->pFlashInstance->p_ctrl);
  
		//if success ok
		if(FSP_SUCCESS == err)
		{
	
			//clear the open flag to indicate memory access is closed.
			pHandle->dataFlashOpenFlag = false;
			
			//
			return true;
			
		}
		
	}
	
	//
	return false;
    
}

/**
  @brief Function to erase the data flash memory.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint32_t const blockAddress start block address to be erased. 
  @oaram uint32_t const numBlocks number of black to be erased.
  @return bool return true if the data memory was correctly reased and false if not
  or a wrong parameter was passed.
*/
bool uCAL_Data_Flash_Erase(DataFlash_Handle_t * pHandle, uint32_t const blockAddress, uint32_t const numBlocks)
{
	
    //verify if the pointer is not null. 
	ASSERT(pHandle != NULL); 
	
	//Verify if the flash memory access was opnned.
	if(pHandle->dataFlashOpenFlag == true)
	{
		
		//Verify if the block address is inside of the data flash memory space and if the
		//number of block to be erased is greater than zero.
		if((blockAddress >= DATA_FLASH_START_ADDRESS) && (blockAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBlocks > 0))
		{
			
			//Verify if the total number of block to be erase are outside of the data flash.
			if(blockAddress + (numBlocks - 1)*DATA_FLASH_BLOCK_OFFSET_64BYTES <= DATA_FLASH_LAST_BLOCK_ADDRESS)
			{
	
				//
				flashEvent = 0xFF;
				
				//Open the flash hp instance. */
				fsp_err_t err = R_FLASH_HP_Erase(pHandle->pFlashInstance->p_ctrl,blockAddress, numBlocks);
  
				//verify if the blank check commmand was sent and if the memory was
				//correctly erased.
				if((FSP_SUCCESS == err) && (uCAL_Is_Flash_Busy(FLASH_TIMEOUT) == FLASH_EVENT_ERASE_COMPLETE))
				{
                    
                    //
                    return true;
                    
				}
				
			}
		
		}
		
	}
	
	//
	return false;
	
}

/**
  @brief Function to write bytes in the data flash memory.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint8_t * data pointer to the data to be written in the data flash memory.
  @param uint32_t const flashAddress start address to be written. 
  @param uint32_t const numBytes number of bytes to be written.
  @return bool return true if the data was correctly written to memory and false if not
  or a wrong parameter was passed.
*/
bool uCAL_Data_Flash_Write(DataFlash_Handle_t * pHandle, uint8_t * data, uint32_t flashAddress, uint32_t const numBytes)
{
    
    //verify if the pointer is not null. 
    ASSERT(pHandle != NULL); 
    
    //Verify if the flash memory access was opnned.
    if(pHandle->dataFlashOpenFlag == true)
    {
        
        //Verify if the flash address is inside of the data flash memory space and if the
        //number of bytes to be written is greater than zero.
        if((flashAddress >= DATA_FLASH_START_ADDRESS) && (flashAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBytes > 0))
        {
            
            //Verify if the total number of bytes to be written are outside of the data flash.
            if(flashAddress + (numBytes - 1) <= DATA_FLASH_END_ADDRESS)
            {
                
                //
                flashEvent = 0xFF;
                
                //Write numBytes in the data flash memory. */
                fsp_err_t err = R_FLASH_HP_Write(pHandle->pFlashInstance->p_ctrl,(uint32_t)data, flashAddress, numBytes);
                
                //verify if the blank check commmand was sent and if the memory was
                //correctly erased.
                if((FSP_SUCCESS == err) && (uCAL_Is_Flash_Busy(FLASH_TIMEOUT) == FLASH_EVENT_WRITE_COMPLETE))
                {
                    
                    //
                    return true;
                    
                }
                
            }
            
        }
        
    }
    
    //
    return false;
}
	
/**
  @brief Function to read bytes in the data flash memory.
  To read bytes from data flash memory, data flash access
  must to be on open state.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint8_t * data pointer to the buffer that will receive data 
  from the data flash memory.
  @param uint32_t const flashAddress start address to be read. 
  @param uint32_t const numBytes number of bytes to be read.
  @return bool true if the data was read and false if not.
*/
bool uCAL_Data_Flash_Read(DataFlash_Handle_t * pHandle, uint8_t * data, uint32_t flashAddress, uint32_t const numBytes)
{
	
	//verify if the pointer is not null. 
	ASSERT(pHandle != NULL); 
	
	//Verify if the flash memory access was opnned.
	if(pHandle->dataFlashOpenFlag == true)
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
                
                //
                return true;
                
			}
			
		}
		
	}
    
    //
    return false;
    
}
	
/**
  @brief Function to performe a blank check in the data flash memory.
  Blanck check is used to verify if the data flash contents was correctly 
  erased(0xFF). Read data from the data flash give a randon value each time 
  you make the read, if the region was erased but no programed(write operation).
  So, the only way to verify if memory was erased if calling the blank check 
  function.
  
  @param FLASH_Handle_t that allow to configure all necessary 
	parameters.
  @param uint32_t const blockAddress start block address to be blank checked. 
  @oaram uint32_t const numBytes number of bytes to be blank checked(at least 4 bytes).
  @return bool return true if the memory was correctly erased and false if not
  or a wrong parameter was passed.
*/
bool uCAL_Data_Flash_Blank_Check(DataFlash_Handle_t * pHandle, uint32_t const blockAddress, uint32_t numBytes)
{
    //verify if the pointer is not null. 
    ASSERT(pHandle != NULL); 
    
    //Verify if the flash memory access was openned.
    if(pHandle->dataFlashOpenFlag == true)
    {
        
        //Verify if the block address is inside of the data flash memory space and if the
        //number of block to be blank checked are greater than four.
        if((blockAddress >= DATA_FLASH_START_ADDRESS) && (blockAddress <= DATA_FLASH_LAST_BLOCK_ADDRESS) && (numBytes > 4))
        {
            
            //Verify if the total number of block to be blank checked are outside of the data flash.
            if(blockAddress + (numBytes - 1) <= DATA_FLASH_LAST_BLOCK_ADDRESS)
            {
                
                //Initalise the variable used to get data flash event.
                flashEvent = 0xFF;
                
                //Blanck check the data flash memory.
                fsp_err_t err = R_FLASH_HP_BlankCheck(pHandle->pFlashInstance->p_ctrl,blockAddress, numBytes,&pHandle->blank_check_result);
                
                //verify if the blank check commmand was sent and if the memory was
                //correctly erased.
                if((FSP_SUCCESS == err) && (uCAL_Is_Flash_Busy(FLASH_TIMEOUT) == FLASH_EVENT_BLANK))
                {
                    
                    //
                    return true;
                    
                }
                
            }
            
        }
        
    }
    
    //
    return false;
}

/**
  * @brief  Interrupt routine to flash events, like write finished and etc.
  * @param  p_args: Flash callback function arguments.
*/
void Flash_IRQHandler(flash_callback_args_t * p_args)  
{
	
    //get the event type.
    flashEvent = p_args->event;
	
}

/******************************************************************
                    Private functions                          
*******************************************************************/

/**
  @brief Function to check interruption response releated to ISR coming
  from the data flash interruption(erased flash, wrote flash, etc).
  This function hold the code until interruption event happens or
  a timeout happened.
  
  @param uint32_t timeout to be used as precaution if the flash
         interruption doesn't happened.
  @return uint8_t returns flashEvent.
*/
static uint8_t uCAL_Is_Flash_Busy(uint32_t timeOut)
{
	
    //Wait until the current flash operation completes
    //or break the loop by timeout.
    for(uint32_t i = 0; i < timeOut; i++)
    {
        
        //different , so interruption event happened
        //break the loop and return event type.
        if(flashEvent != 0xFF)
        {
            
            //
            break;
            
        }
        
    } 
    
    //
    return flashEvent;
    
}
