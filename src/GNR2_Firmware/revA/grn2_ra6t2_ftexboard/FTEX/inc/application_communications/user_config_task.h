/**
* @file   user_config_task.h
* @author Bruno Alves
* @brief  Application file used to read, write, receive 
          and send user configuration data.
*
*
*/

#ifndef USER_CONFIG_TASK_H_
#define USER_CONFIG_TASK_H_

/*********************************************
               Includes                       
*********************************************/

#include "uCAL_DATAFLASH.h"
#include "user_config_data.h"
#include "comm_config.h"
#include "vc_parameters.h"


/*********************************************
                Defines
*********************************************/

#define NUMBER_OF_BYTES_IN_THE_BLOCK    64U

//first byte used to show that data memory
//has user configuration
#define ID0_DATA_FLASH                 0xB5

//second byte used to show that data memory
//has user configuration
#define ID1_DATA_FLASH                 0xC3


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
  @brief Function to verify if data flash memory is empty or not.
	If it's empty write default configuration in the data memory.
    If it's not empty read the configuration from the data memory.
    NOTE: on this first version this function will be used to test 
    the task:
     - Move PAS configurable fields to flash memory
     - Starting with a defaut value value, that will be hold
       forever in data memory, and a dynamic value that will increased
       every time the system reset.
  
  @param void
  @return void
*/
void InitUserConfig_FromDataFLASH(void);
    
/**
  @brief Function to pass the address(pointer)
  of the userConfigData struct.
  
  @param void
  @return void
  struct.
*/
void updateUserConfigData(void);

/**
  @brief Function to get PasAlgorithm
  read from data flash memory.
  
  @param void
  @return uint8_t a number que represent pas algorithm type.
*/
uint8_t getPasAlgorithm(void);

/**
  @brief Function to get number Of Pas Levels
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent the number Of Pas Levels.
*/
uint8_t getNumberPasLevels(void);

/**
  @brief Function to get Pas Max Power
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Pas Max Power
  on %(0 until 100).
*/
uint8_t getPasMaxPower(void);

/**
  @brief Function to get Torque Minimum Threshold
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
uint8_t getTorqueMinimumThreshold(void);

/**
  @brief Function to get Torque Sensor Multiplier
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Sensor Multiplier
  between 1 and 3.
*/
uint8_t getTorqueSensorMultiplier(void);

#endif