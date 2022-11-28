/**
* @file   user_ConfigTask.h
* @author Bruno Alves
* @brief  Application file used to read, write, receive 
          and send user configuration data.
*
*
*/

#ifndef USER_CONFIGTASK_H_
#define USER_CONFIGTASK_H_

/*********************************************
               Includes                       
*********************************************/

#include "uCal_DataFlash.h"

/*********************************************
                Defines
*********************************************/

#define NUMBER_OF_BYTES_TO_BE_READ    64
#define ID_DATA_FLASH_0           0xB5
#define ID_DATA_FLASH_1           0xC7


/*********************************************
          Data Struct Definition
*********************************************/


//enum used to control the user configuration 
//state machine.
typedef enum
{
	
	IDLE,
	OPEN,
	CLOSE,
	READ,
	ERASE,
	WRITE,
	ERROR,
	
} UserConfigStateMachine_t;


// ==================== Public function prototypes ========================= //

/**
  @brief Function to control data flash memory access.
	This function user the uCal data flash library to 
	interact with the data flash memory inside of the 
	RA6T2 microcontroller.
  
  @param void
  @return void
*/
void UserConfigStateMachine(void);

#endif