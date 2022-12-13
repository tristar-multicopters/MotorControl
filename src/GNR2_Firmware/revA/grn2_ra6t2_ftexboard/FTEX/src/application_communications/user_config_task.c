/**
*  @file user_config_task.c
*  @brief Application layer to receive and send user configuration 
*  using the uCAL_DATA_FLASH.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "user_config_task.h"
#include "ASSERT_FTEX.h"
#include "vc_config.h"

/*********************************************
                Defines
*********************************************/
#define MAX_ERASE_ATTEMPTS     3

//definition used to control how many
//blocks of the data flash memory are being
//used to hold the user configuration.
//each block has 64 bytes.
#define NUMBER_OF_BLOCKS_USED  1

/*********************************************
                Private Variables
*********************************************/

//control typedef enum initialised at 
static UserConfigStateMachine_t userConfigStateMachine = IDLE;

//struct used to hold the default value of the user configuration.
static User_ConfigData_t userConfigData = 
{ 
    .dataHeader[0] = ID0_DATA_FLASH,
    .dataHeader[1] = ID1_DATA_FLASH,
    .vehicle = VEHICLE_SELECTION,
    .PAS_ConfigData.pasAlgorithm = PAS_TORQUE_USE,
    .PAS_ConfigData.numberOfPasLevels = PAS_MAX_LEVEL,
    .PAS_ConfigData.pasMaxPower = PAS_MAX_TORQUE_RATIO,
    .PAS_ConfigData.torqueMinimumThreshold = PTS_OFFSET_PTS2TORQUE,
    .PAS_ConfigData.torqueSensorMultiplier = PAS_LEVEL_COEFF,
    .PAS_ConfigData.cadenceHybridLeve1Speed = PAS_LEVEL_SPEED_0,
    .PAS_ConfigData.cadenceHybridLeve2Speed = PAS_LEVEL_SPEED_1,
    .PAS_ConfigData.cadenceHybridLeve3Speed = PAS_LEVEL_SPEED_2,
    .PAS_ConfigData.cadenceHybridLeve4Speed = PAS_LEVEL_SPEED_3,
    .PAS_ConfigData.cadenceHybridLeve5Speed = PAS_LEVEL_SPEED_4,
    .PAS_ConfigData.cadenceHybridLeve6Speed = PAS_LEVEL_SPEED_5,
    .PAS_ConfigData.cadenceHybridLeve7Speed = PAS_LEVEL_SPEED_0,
    .PAS_ConfigData.cadenceHybridLeve8Speed = PAS_LEVEL_SPEED_1,
    .PAS_ConfigData.cadenceHybridLeve9Speed = PAS_LEVEL_SPEED_2,
    .PAS_ConfigData.cadenceHybridLeve10Speed = PAS_LEVEL_SPEED_3,
    .Throttle_ConfigData.maxSpeed = PAS_MAX_KM_SPEED,
    .Throttle_ConfigData.walkModeSpeed = PAS_WALKMODE_OVER_THROTTLE,

};

//struct used to hold the values of the user configuration
//read from the data memory.
//static User_ConfigData_t userConfigData;

//buffer used to 
static uint8_t data[NUMBER_OF_BYTES_IN_THE_BLOCK];

/*********************************************
                Public Variables
*********************************************/


// ==================== Public Functions ======================== //
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
void InitUserConfig_FromDataFLASH(void)
{
    //variable used to control how my attempts will be done
    //when trying to erase data flash memory.
    uint8_t attempts;
    
    //try to open flash memory
    if(uCAL_Flash_Open(&DataFlashHandle) == true)
    {
        
        //get date from data flash memory(backup)
        uCAL_Data_Flash_Read(&DataFlashHandle, data, FLASH_HP_DF_BLOCK_0, USER_DATA_CONFIG_LENGTH);
        
        //if the data id(header of the user data) or the bike type is are not the same
        //clear data flash memory.
        if ((ID0_DATA_FLASH != data[0]) || (ID1_DATA_FLASH != data[1]) || (userConfigData.vehicle != VEHICLE_SELECTION))
        {
            
            //try max three times to erase data flash memory.
            for(attempts = 0; attempts < MAX_ERASE_ATTEMPTS ; attempts++)
            {
            
                //erase one block(block 0) of the data flash memory before write on it.
                //if you need more than 64 to write user configuration we need to
                //erase more than 1 block.
                uCAL_Data_Flash_Erase(&DataFlashHandle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BLOCKS_USED);
            
                //check if all bytes were erased.
                //we are using just 1 block, so just 64 are checked.
                //to check 2 or more blocks use n*NUMBER_OF_BYTES_IN_THE_BLOCK.
                if (uCAL_Data_Flash_Blank_Check(&DataFlashHandle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
                {
                    
                    //if memory was correctly erased break
                    //stop the attempts.
                    break;
                
                }
                
            }
            
            //check if the memory was erased in less than three attempts
            //if yes, write the new user configuration on it.
            if (attempts < MAX_ERASE_ATTEMPTS )
            {
                
                //copy all user configuration values to the buffer.
                memcpy(data,&userConfigData,USER_DATA_CONFIG_LENGTH);
                
                //write user configuration that represents USER_DATA_CONFIG_LENGTH bytes.
                uCAL_Data_Flash_Write(&DataFlashHandle,data,FLASH_HP_DF_BLOCK_0, USER_DATA_CONFIG_LENGTH);
                
            }
            
        }
        else
        {
            
           //copy all user configuration values to the userConfigData struct.
           memcpy(&userConfigData,data,USER_DATA_CONFIG_LENGTH); 
            
        }
        
    }
    
    //Close data flash memory access.
    uCAL_Flash_Close(&DataFlashHandle);
    
}

/**
  @brief Function to update the userConfigData read
  from the flash memory. This function must to be called before
  MC_BootUp() function.
  
  @param void
  @return void
*/
void updateUserConfigData(void)
{
    
    //update PAS_TORQUE_USE
    VCInterfaceHandle.pPowertrain->pPAS->sParameters.bTorqueSensorUse = getPasAlgorithm();
    
    //update PAS_MAX_LEVEL
    VCInterfaceHandle.pPowertrain->pPAS->sParameters.bMaxLevel = getNumberPasLevels();
    
    //update PAS_MAX_TORQUE_RATIO
    VCInterfaceHandle.pPowertrain->pPAS->sParameters.hMaxTorqueRatio = getPasMaxPower();
    
    //update PTS_OFFSET_PTS2TORQUE
    VCInterfaceHandle.pPowertrain->pPAS->pPTS->hParameters.hOffsetMT = getTorqueMinimumThreshold();    
}

/**
  @brief Function to get PasAlgorithm
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent pas algorithm type.
*/
uint8_t getPasAlgorithm(void)
{
    return userConfigData.PAS_ConfigData.pasAlgorithm;   
}

/**
  @brief Function to get number Of Pas Levels
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent the number Of Pas Levels.
*/
uint8_t getNumberPasLevels(void)
{
    return userConfigData.PAS_ConfigData.numberOfPasLevels; 
}

/**
  @brief Function to get Pas Max Power
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Pas Max Power
  on %(0 until 100).
*/
uint8_t getPasMaxPower(void)
{
    return userConfigData.PAS_ConfigData.pasMaxPower; 
}

/**
  @brief Function to get Torque Minimum Threshold
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
uint8_t getTorqueMinimumThreshold(void)
{
    return userConfigData.PAS_ConfigData.torqueMinimumThreshold; 
}

/**
  @brief Function to get Torque Sensor Multiplier
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Sensor Multiplier
  between 1 and 3.

NOTE: parameter passed in the PedalAssist_GetTorqueFromTS
*/
uint8_t getTorqueSensorMultiplier(void)
{
    return userConfigData.PAS_ConfigData.torqueSensorMultiplier; 
}