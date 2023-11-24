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

// disable warning about user_config_data modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_data.h"
#pragma clang diagnostic pop

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

//definition used to control how many
//blocks of the data flash memory are being
//used to hold the user configuration.
//each block has 64 bytes.
#define NUMBER_OF_BLOCKS_USED             7

//code used to indicate that data flash
//memory(user configuration) is being updated by an external
//device.
//This value must be write in the respective 
//Object dictionary ID(OD-ID) to disable theses variables
//to be upadted by the application using UpdateObjectDictionnary().
#define KEY_USER_DATA_CONFIG_BEING_UPDATED   0xD5A3

//code used to indicate that data flash
//memory was updated by an external
//device.
//This value must be write in the respective 
//Object dictionary ID(OD-ID) to disable theses variables
//to be upadted by the application using UpdateObjectDictionnary()
//and to trigger a task responsible to write the values received 
//in the data flash memory and reset the system.
#define KEY_USER_DATA_CONFIG_UPDATED         0xC2E5

//CRC-16-CCITT polynom
#define CCITT_POLYNOM 0x1021


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

//
typedef struct 
{
    
    VCI_Handle_t  *pVController;
    DataFlash_Handle_t *pDataFlash_Handle;
    
}UserConfigHandle_t;

//========================= EXTERN TYPES ==========================//

extern UserConfigHandle_t UserConfigHandle;


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
  
  @param DataFlash_Handle_t * pDataFlashHandle pointer to the struct that control
         data flash init.
  @param VCI_Handle_t *pVCIHandle pointer to the struct that controls the vehicle interface
  @return void
*/
void UserConfigTask_InitUserConfigFromDataFlash(UserConfigHandle_t * userConfigHandle, DataFlash_Handle_t * pDataFlashHandle, VCI_Handle_t *pVCIHandle);

/**
  @brief Function to write user data config into the data flash memory.
  @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
  to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  @return void
*/
void UserConfigTask_WriteUserConfigIntoDataFlash(UserConfigHandle_t * userConfigHandle);
    
/**
  @brief Function to update the userConfigData read
  from the flash memory. This function must to be called before
  MC_BootUp() function.
  
  @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
  to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  @return void
*/
void UserConfigTask_UpdateUserConfigData(UserConfigHandle_t * userConfigHandle);

/**
  @brief Function to get PasAlgorithm
  read from data flash memory.
  
  @param void
  @return uint8_t a number que represent pas algorithm type.
*/
uint8_t UserConfigTask_GetPasAlgorithm(void);

/**
  @brief Function to update PasAlgorithm value
  read from data flash memory.
  
  @param uint8_t value to be passed into the pasAlgorithm  variable.
  @return void
*/
void UserConfigTask_UpdataPasAlgorithm(uint8_t value);

/**
  @brief Function to get number Of Pas Levels
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent the number Of Pas Levels.
*/
uint8_t UserConfigTask_GetNumberPasLevels(void);

/**
  @brief Function to update number Of Pas Levels value
  read from data flash memory.
  
  @param uint8_t value to be passed into the number Of Pas Levels
  @return void
*/
void UserConfigTask_UpdateNumberPasLevels(uint8_t value);

/**
  @brief Function to get Pas Max Power
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Pas Max Power
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetPasMaxPower(void);

/**
  @brief Function to update Pas Max Power value
  read from data flash memory.
  
  @param uint8_t
  @return  void
*/
void UserConfigTask_UpdatePasMaxPower(uint8_t value);

/**
  @brief Function to update Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Minimum Threshold Startup
  @return void
 
*/
uint8_t UserConfigTask_GetTorqueMinimumThresholdStartup(void);

/**
  @brief Function to get Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
void UserConfigTask_UpdateTorqueMinimumThresholdStartup(uint8_t value);

/**
  @brief Function to get Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Startup Offset Minimum Threshold Speed
  in RPM.
*/
uint8_t UserConfigTask_GetStartupOffsetMinimumThresholdSpeed(void);

/**
  @brief Function to update Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param uint8_t value to be passed into the Startup Offset Minimum Threshold Speed
  @return void
 
*/
void UserConfigTask_UpdateStartupOffsetMinimumThresholdSpeed(uint8_t value);

/**
  @brief Function to get Torque Minimum Threshold
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetTorqueMinimumThreshold(void);

/**
  @brief Function to update Torque Minimum Threshold value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Minimum Threshold
  @return void
 
*/
void UserConfigTask_UpdateTorqueMinimumThreshold(uint8_t value);

/**
  @brief Function to get Torque Sensor Multiplier
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Sensor Multiplier
  between 1 and 3.
*/
uint8_t UserConfigTask_GetTorqueSensorMultiplier(void);

/**
  @brief Function to update Torque Sensor Multiplier value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Sensor Multiplier
  @return void

*/
void UserConfigTask_UpdateTorqueSensorMultiplier(uint8_t value);

/**
  @brief Function to get torque Max Speed
  read from data flash memory.
  
  @param void
  @return uint8_t !!!!range will be defined.

*/
uint8_t UserConfigTask_GetTorqueMaxSpeed(void);

/**
  @brief Function to update torque Max Speed avlue
  read from data flash memory.
  
  @param uint8_t value to be passed into the
  @return void

*/
void UserConfigTask_UpdateTorqueMaxSpeed(uint8_t value);

/**
  @brief Function to get cadence Level Speed
  read from data flash memory.
  
  @param void
  @return uint8_t speed up to which this PAS level will give motor assistance
          range bewteen 0-40.

*/
uint8_t UserConfigTask_GetCadenceLevelSpeed(uint8_t pasLevel);

/**
  @brief Function to update cadence Level Speed value
  read from data flash memory.
  
  @param uint8_t value to be passed into the
  @return void

*/
void UserConfigTask_UpdateCadenceLevelSpeed(uint8_t pasLevel, uint8_t value);

/**
  @brief Function to get torqueLevelPower
  read from data flash memory.
  
  @param void
  @return uint8_t Percentage of max motor assistance that this PAS level will give
          range bewteen 0-x.

*/
uint8_t UserConfigTask_GetTorqueLevelPower(uint8_t pasLevel);

/**
  @brief Function to update torqueLevelPower value
  read from data flash memory.
  
  @param uint8_t value to be passed into the torqueLevelPower
  @return void

*/
void UserConfigTask_UpdateTorqueLevelPower(uint8_t pasLevel, uint8_t value);

/**
  @brief Function to get bike max speed
  read from data flash memory.
  
  @param void
  @return uint8_t The max speed that the throttle will bring the vehicle to.
          range bewteen 0-75.

*/
uint8_t UserConfigTask_GetBikeMaxSpeed(void);

/**
  @brief Function to update bike max speed value
  read from data flash memory.
  
  @param uint8_t value to be passed into the bike max speed
  @return void

*/
void UserConfigTask_UpdateBikeMaxSpeed(uint8_t value);

/**
  @brief Function to get bike speed on walk mode
  read from data flash memory.
  
  @param void
  @return uint8_t Speed that the walk mode of the vehicle goes up to.
          range bewteen 0-10.

*/
uint8_t UserConfigTask_GetWalkModeSpeed(void);

/**
  @brief Function to update bike speed on walk mode value
  read from data flash memory.
  
  @param uint8_t value to be passed into the walk mode speed
  @return void

*/
void UserConfigTask_UpdateWalkModeSpeed(uint8_t value);

/**
  @brief Function to get Wheel Diameter
  read from data flash memory.
  
  @param void
  @return uint8_t Wheel Diameter

*/
uint8_t UserConfigTask_GetWheelDiameter(void);

/**
  @brief Function to update bike Wheel Diameter value
  read from data flash memory.
  
  @param uint8_t value to be passed into the WheelDiameter
  @return void

*/
void UserConfigTask_UpdateWheelDiameter(uint8_t value);

/**
  @brief Function to get Screen Protocol
  read from data flash memory.
  
  @param void
  @return uint8_t Wheel Diameter

*/
uint8_t UserConfigTask_GetScreenProtocol(void);

/**
  @brief Function to update bike Screen Protocol value
  read from data flash memory.
  
  @param uint8_t value to be passed into the WheelDiameter
  @return void

*/
void UserConfigTask_UpdateScreenProtocol(uint8_t value);

/**

  @brief Function used to calculate a CRC 16 type using the same polynom 
  used by the bluetooth protocol.CCITT 16 bits polynom.
  
  @param uint8_t * data pointer to the data buffer where the CRC must be done.
  @param uint8_t length the length of the data to be calculated by the CRC algorithm.
  @return uint16_t return the calculated CRC.

*/
uint16_t UserConfigTask_CalculateCRC(uint8_t * buffer, uint8_t length);

#endif