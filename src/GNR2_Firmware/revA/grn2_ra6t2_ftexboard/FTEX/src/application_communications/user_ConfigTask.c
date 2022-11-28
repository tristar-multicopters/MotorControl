/**
*  @file user_config_task.c
*  @brief Application layer to receive and send user configuration 
*  using the uCAL_DATA_FLASH.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "user_ConfigTask.h"
#include "ASSERT_FTEX.h"
#include "comm_config.h"

/*********************************************
                Defines
*********************************************/


/*********************************************
                Private Variables
*********************************************/

//control typedef enum initialised at 
static UserConfigStateMachine_t userConfigStateMachine = IDLE;

//
static uint8_t data[64];

/*********************************************
                Public Variables
*********************************************/


// ==================== Public Functions ======================== //

/**
  @brief Function to control data flash memory access.
	This function user the uCal data flash library to 
	interact with the data flash memory inside of the 
	RA6T2 microcontroller.
  
  @param void
  @return void
*/
void UserConfigStateMachine(void)
{
    //
    uint8_t i,j;
    
    //just to test.
    for(i = 0;i < 10; i++)
    {
        
        //State machine that controls the access to the data flash
        //memory.
        switch(userConfigStateMachine)
        {
            
            //stay in idle state and verify if the 
            //flash access is openned or not to
            //move to the next state.
            case IDLE:
                
                //if flash access is not open, move to the open state.
                //if it's open move to the read state to get data from
                // the data flash memory.
                if(DataFlashHandle.dataFlashOpenFlag == false)
                {
                    
                    //Move to open state.
                    userConfigStateMachine = OPEN;
                    
                }
                else
                {
                    
                    //Move to open state.
                    userConfigStateMachine = READ;
                    
                }
                
            break;
                
            //Try to open the the access to the data flash memory.
            case OPEN:
                
                //
                if(uCAL_Flash_Open(&DataFlashHandle) == true)
                {
                    
                    //
                    userConfigStateMachine = READ;
                    
                }
                else
                {
                    
                    //
                    userConfigStateMachine = ERROR;
                    
                }
                
            break;
                
            //
            case READ:
                
                //set buffer with zeros.
                memset(data,0,NUMBER_OF_BYTES_TO_BE_READ);
            
                //get date from data flash memory(backup)
                uCAL_Data_Flash_Read(&DataFlashHandle, data, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_TO_BE_READ);
            
                //
                userConfigStateMachine = ERASE;
            
            break;
            
            //
            case ERASE:
                
                //
                uCAL_Data_Flash_Erase(&DataFlashHandle, FLASH_HP_DF_BLOCK_0, 1);
            
                //
                uCAL_Data_Flash_Blank_Check(&DataFlashHandle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_TO_BE_READ);
                
                //
                if(uCAL_Data_Flash_Blank_Check(&DataFlashHandle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_TO_BE_READ) == true)
                {
                    
                    //
                    userConfigStateMachine = WRITE;
                    
                }
                else
                {
                    
                    //
                    userConfigStateMachine = ERROR;
                    
                }
                
            break;
                
            //
            case WRITE:
                
                //
                if((ID_DATA_FLASH_0 != data[0]) || (ID_DATA_FLASH_1 != data[1]))
                {
                    
                    //set buffer with zeros.
                    memset(data,0,NUMBER_OF_BYTES_TO_BE_READ);
                    
                    //pass data flash id code(memory was configured).
                    data[0] = ID_DATA_FLASH_0;
                    data[1] = ID_DATA_FLASH_1;
                    
                }
                else
                {
                    
                    //
                    for(j = 2;j < NUMBER_OF_BYTES_TO_BE_READ; j++)
                    {
                        
                        //
                        data[j]++;
                        
                    }
                    
                }
                
                //
                uCAL_Data_Flash_Write(&DataFlashHandle,data,FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_TO_BE_READ);
                
                //
                userConfigStateMachine = CLOSE;
                
            break;
                
            //
            case CLOSE:
                
                //
                uCAL_Flash_Close(&DataFlashHandle);
            
                //
                userConfigStateMachine = ERROR;
            
            break;
            
            //
            case ERROR:
                
            break;
            
        }		
        
        //
        if(userConfigStateMachine == ERROR)
        {
            
            break;
            
        }
        
    }
    
}
