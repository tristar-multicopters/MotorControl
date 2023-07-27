/**
*  @file user_config_task.c
*  @brief Application layer to receive and send user configuration 
*  using the uCAL_DATA_FLASH.
*/ 

/*********************************************
                Includes                       
*********************************************/
// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop

#include "ASSERT_FTEX.h"
#include "vc_config.h"

/*********************************************
                Defines
*********************************************/
#define MAX_ERASE_ATTEMPTS     3



/*********************************************
                Private Variables
*********************************************/

//struct used to hold the default value of the user configuration.
static User_ConfigData_t userConfigData = 
{ 
    .dataHeader[0] = ID0_DATA_FLASH,
    .dataHeader[1] = ID1_DATA_FLASH,
    .vehicle = VEHICLE_SELECTION,
    .PAS_ConfigData.pasAlgorithm = PAS_ALGORITHM,
    .PAS_ConfigData.numberOfPasLevels = PAS_MAX_LEVEL,
    .PAS_ConfigData.pasMaxPower = PAS_MAX_TORQUE_RATIO,
    .PAS_ConfigData.torqueMinimumThreshold = PTS_OFFSET_PTS2TORQUE, 
    .PAS_ConfigData.torqueMinimumThresholdStartup = PTS_OFFSET_PTS2TORQUE_STARTUP,
    .PAS_ConfigData.startupTorqueMinimumThresholdSpeed = PTS_OFFSET_STARTUP_SPEED_KMH,
    .PAS_ConfigData.torqueSensorMultiplier = PAS_TORQUE_GAIN,
    .PAS_ConfigData.torqueMaxSpeed = 0,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_0] = PAS_LEVEL_SPEED_0,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_1] = PAS_LEVEL_SPEED_1,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_2] = PAS_LEVEL_SPEED_2,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_3] = PAS_LEVEL_SPEED_3,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_4] = PAS_LEVEL_SPEED_4,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_5] = PAS_LEVEL_SPEED_5,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_6] = PAS_LEVEL_SPEED_6,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_7] = PAS_LEVEL_SPEED_7,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_8] = PAS_LEVEL_SPEED_8,
    .PAS_ConfigData.cadenceHybridLeveSpeed[PAS_9] = PAS_LEVEL_SPEED_9,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_0] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_1] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_2] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_3] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_4] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_5] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_6] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_7] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_8] = 0,
    .PAS_ConfigData.torqueLevelPower[TORQUE_LEVEL_9] = 0,
    .Throttle_ConfigData.maxSpeed = PAS_MAX_KM_SPEED,
    .Throttle_ConfigData.walkModeSpeed = PAS_LEVEL_SPEED_WALK,
    .crc = 0x0000,

};

//struct used to hold the values of the user configuration
//read from the data memory.
//static User_ConfigData_t userConfigData;

//buffer used to 
static uint8_t data[NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN];

/*********************************************
                Public Variables
*********************************************/

//Handle to access DataFlash_Handle_t and VCI_Handle_t.
UserConfigHandle_t UserConfigHandle;

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
  
  @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
         to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  @param DataFlash_Handle_t * pDataFlashHandle pointer to the struct that control
         data flash init.
  @param VCI_Handle_t *pVCIHandle pointer to the struct that controls the vehicle interface
  @return void
*/
void UserConfigTask_InitUserConfigFromDataFlash(UserConfigHandle_t * userConfigHandle, DataFlash_Handle_t * pDataFlashHandle, VCI_Handle_t *pVCIHandle)
{
    
    //verify the pointers.
    ASSERT(userConfigHandle != NULL);
    ASSERT(pDataFlashHandle != NULL);
    ASSERT(pVCIHandle != NULL);
    
    //variable used to control how my attempts will be done
    //when trying to erase data flash memory.
    uint8_t attempts;
    
    //variable used to receive the crc 
    uint16_t crc = 0x0000;
    
    //pass the pointer that control data flash
    userConfigHandle->pDataFlash_Handle = pDataFlashHandle;
    //pass the pointer that control vehicle interface
    //this pointer must be passed here because it will be used
    //for other functions in this file.
    userConfigHandle->pVController = pVCIHandle;
    
    //try to open flash memory
    if(uCAL_Flash_Open(userConfigHandle->pDataFlash_Handle) == true)
    {   
        //get date from data flash memory(backup)
        uCAL_Data_Flash_Read(userConfigHandle->pDataFlash_Handle, data, FLASH_HP_DF_BLOCK_0, USER_DATA_CONFIG_LENGTH);
        
        //caclualte the crc to verify integrit of the data excluding crc bytes.
        crc = UserConfigTask_CalculateCRC(data, USER_DATA_CONFIG_LENGTH - 2);
        
        //Check data header, bike type and crc.if they one of them are different 
        //clear data flash memory.
        if ((ID0_DATA_FLASH != data[0]) || (ID1_DATA_FLASH != data[1]) || (data[2] != VEHICLE_SELECTION) || 
           (crc != (uint16_t)((data[USER_DATA_CONFIG_LENGTH - 1] << 8) + data[USER_DATA_CONFIG_LENGTH - 2])))
        {
            //try max three times to erase data flash memory.
            for(attempts = 0; attempts < MAX_ERASE_ATTEMPTS ; attempts++)
            {
                //erase one block(block 0) of the data flash memory before write on it.
                //if you need more than 64 to write user configuration we need to
                //erase more than 1 block.
                uCAL_Data_Flash_Erase(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BLOCKS_USED);
            
                //check if all bytes were erased.
                //we are using just 1 block, so just 64 are checked.
                //to check 2 or more blocks use n*NUMBER_OF_BYTES_IN_THE_BLOCK.
                if (uCAL_Data_Flash_Blank_Check(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
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
                //caclualte the crc to verify integrit of the data excluding crc bytes.
                crc = UserConfigTask_CalculateCRC((uint8_t *)&userConfigData, USER_DATA_CONFIG_LENGTH - 2);
            
                //get the new crc.
                userConfigData.crc = crc;
                
                //copy all user configuration values to the buffer.
                memcpy(data,&userConfigData,USER_DATA_CONFIG_LENGTH);
                
                //write user configuration that represents USER_DATA_CONFIG_LENGTH bytes.
                uCAL_Data_Flash_Write(userConfigHandle->pDataFlash_Handle,data,FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN); 
            }   
        }
        else
        {
            
           //copy all user configuration values to the userConfigData struct.
           memcpy(&userConfigData,data,USER_DATA_CONFIG_LENGTH); 
            
        }
    }
    
    //Close data flash memory access.
    uCAL_Flash_Close(userConfigHandle->pDataFlash_Handle);
}

/**
  @brief Function to write user data config into the data flash memory.
  @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
         to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  @return void
*/
void UserConfigTask_WriteUserConfigIntoDataFlash(UserConfigHandle_t * userConfigHandle)
{
    
    //verify the pointer.
    ASSERT(userConfigHandle->pDataFlash_Handle != NULL);
    
    //variable used to control how my attempts will be done
    //when trying to erase data flash memory.
    uint8_t attempts;
    
    //variable used to receive the crc 
    uint16_t crc = 0x0000;
    
    //try to open flash memory
    if (uCAL_Flash_Open(userConfigHandle->pDataFlash_Handle) == true)
    {
            
        //try max three times to erase data flash memory.
        for(attempts = 0; attempts < MAX_ERASE_ATTEMPTS ; attempts++)
        {
            
            //erase one block(block 0) of the data flash memory before write on it.
            //if you need more than 64 to write user configuration we need to
            //erase more than 1 block.
            uCAL_Data_Flash_Erase(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BLOCKS_USED);
            
            //check if all bytes were erased.
            //we are using just 1 block, so just 64 are checked.
            //to check 2 or more blocks use n*NUMBER_OF_BYTES_IN_THE_BLOCK.
            if (uCAL_Data_Flash_Blank_Check(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
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
            
            //caclualte the crc to verify integrit of the data excluding crc bytes.
            crc = UserConfigTask_CalculateCRC((uint8_t *)&userConfigData, USER_DATA_CONFIG_LENGTH - 2);
            
            //get the new crc.
            userConfigData.crc = crc;
            
            //copy all user configuration values to the buffer.
            memcpy(data,&userConfigData,USER_DATA_CONFIG_LENGTH);
                
            //write user configuration that represents USER_DATA_CONFIG_LENGTH bytes.
            uCAL_Data_Flash_Write(userConfigHandle->pDataFlash_Handle,data,FLASH_HP_DF_BLOCK_0, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN);
                
        }
            
    }
    
    //Close data flash memory access.
    uCAL_Flash_Close(userConfigHandle->pDataFlash_Handle);
    
    //do a software reset to update the system with the new user configuration.
    NVIC_SystemReset();
    
}

/**
  @brief Function to update the userConfigData read
  from the flash memory. This function must to be called before
  MC_BootUp() function.
  
  @param UserConfigHandle_t * userConfigHandle pointer to the struct responsible
  to give access to other structs like DataFlash_Handle_t and VCI_Handle_t.
  @return void
*/
void UserConfigTask_UpdateUserConfigData(UserConfigHandle_t * userConfigHandle)
{
 
    //verify the pointer.
    ASSERT(userConfigHandle->pVController  != NULL);
    
    //update userConfigData.PAS_ConfigData.pasAlgorithm(PAS_ALGORITHM)
    userConfigHandle->pVController->pPowertrain->pPAS->bCurrentPasAlgorithm = UserConfigTask_GetPasAlgorithm();
    
    //update userConfigData.PAS_ConfigData.numberOfPasLevels(PAS_MAX_LEVEL)
    userConfigHandle->pVController->pPowertrain->pPAS->sParameters.bMaxLevel = UserConfigTask_GetNumberPasLevels();
    
    //update userConfigData.PAS_ConfigData.pasMaxPower(PAS_MAX_TORQUE_RATIO)
    userConfigHandle->pVController->pPowertrain->pPAS->sParameters.hMaxTorqueRatio = UserConfigTask_GetPasMaxPower();

    //update userConfigData.PAS_ConfigData.torqueMinimumThresholdStartup(PTS_OFFSET_PTS2TORQUE_STARTUP)
    userConfigHandle->pVController->pPowertrain->pPAS->pPTS->hParameters.hOffsetMTStartup = UserConfigTask_GetTorqueMinimumThresholdStartup();

    //update userConfigData.PAS_ConfigData.startupTorqueMinimumThresholdSpeed(PTS_OFFSET_STARTUP_SPEED_KMH)
    userConfigHandle->pVController->pPowertrain->pPAS->pPTS->hParameters.hStartupOffsetMTSpeedKMH = UserConfigTask_GetStartupOffsetMinimumThresholdSpeed();
    
    //update userConfigData.PAS_ConfigData.torqueMinimumThreshold(PTS_OFFSET_PTS2TORQUE)
    userConfigHandle->pVController->pPowertrain->pPAS->pPTS->hParameters.hOffsetMT = UserConfigTask_GetTorqueMinimumThreshold();  

    //update PAS_ConfigData.torqueSensorMultiplier(PAS_TORQUE_GAIN)    
    userConfigHandle->pVController->pPowertrain->pPAS->sParameters.bTorqueGain = UserConfigTask_GetTorqueSensorMultiplier();
    
    //update PAS_ConfigData.torqueMaxSpeed(will be defined).
    
    
    //PAS_ConfigData.cadenceHybridLeveSpeed parameter is not passed to any system variable on the inialization.
    
    //update .PAS_ConfigData.torqueLevelPower(will be define)
    
    //update Throttle_ConfigData.maxSpeed(PAS_MAX_KM_SPEED).
    userConfigHandle->pVController->pPowertrain->pPAS->sParameters.hPASMaxKmSpeed = UserConfigTask_GetBikeMaxSpeed();
    
    //Throttle_ConfigData.walkMOdeSpeed(PAS_LEVEL_SPEED_WALK) is not passed
    //directly to any variable. Because of this is not updated here.
  
}

/**
  @brief Function to get PasAlgorithm
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent pas algorithm type.
*/
uint8_t UserConfigTask_GetPasAlgorithm(void)
{
    return userConfigData.PAS_ConfigData.pasAlgorithm;   
}

/**
  @brief Function to update PasAlgorithm value
  read from data flash memory.
  
  @param uint8_t value to be passed into the pasAlgorithm  variable.
  @return void
*/
void UserConfigTask_UpdataPasAlgorithm(uint8_t value)
{
    userConfigData.PAS_ConfigData.pasAlgorithm = value;   
}

/**
  @brief Function to get number Of Pas Levels
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent the number Of Pas Levels.
*/
uint8_t UserConfigTask_GetNumberPasLevels(void)
{
    return userConfigData.PAS_ConfigData.numberOfPasLevels; 
}

/**
  @brief Function to update number Of Pas Levels value
  read from data flash memory.
  
  @param uint8_t value to be passed into the number Of Pas Levels
  @return void
*/
void UserConfigTask_UpdateNumberPasLevels(uint8_t value)
{
    //verify if vakue is in the range.
    if((value <= 10) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.numberOfPasLevels = value;
    }        
}

/**
  @brief Function to get Pas Max Power
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Pas Max Power
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetPasMaxPower(void)
{
    return userConfigData.PAS_ConfigData.pasMaxPower; 
}

/**
  @brief Function to update Pas Max Power value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Pas Max Power
  @return  void
*/
void UserConfigTask_UpdatePasMaxPower(uint8_t value)
{
    //verify if vakue is in the range.
    if((value <= 100) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.pasMaxPower = value;    
    }
}

/**
  @brief Function to get Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetTorqueMinimumThresholdStartup(void)
{
    return userConfigData.PAS_ConfigData.torqueMinimumThresholdStartup;     
}

/**
  @brief Function to update Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Minimum Threshold Startup
  @return void
 
*/
void UserConfigTask_UpdateTorqueMinimumThresholdStartup(uint8_t value)
{
    //verify if value is in the range.
    if((value <= 100) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.torqueMinimumThresholdStartup = value;
    }        
}

/**
  @brief Function to get Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Startup Offset Minimum Threshold Speed
  in RPM.
*/
uint8_t UserConfigTask_GetStartupOffsetMinimumThresholdSpeed(void)
{
    return userConfigData.PAS_ConfigData.startupTorqueMinimumThresholdSpeed; 
}

/**
  @brief Function to update Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param uint8_t value to be passed into the Startup Offset Minimum Threshold Speed
  @return void
 
*/
void UserConfigTask_UpdateStartupOffsetMinimumThresholdSpeed(uint8_t value)
{
    //verify if value is in the range.
    if(value >= 0)
    {
        userConfigData.PAS_ConfigData.startupTorqueMinimumThresholdSpeed = value;
    }        
}

/**
  @brief Function to get Torque Minimum Threshold
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetTorqueMinimumThreshold(void)
{
    return userConfigData.PAS_ConfigData.torqueMinimumThreshold; 
}

/**
  @brief Function to update Torque Minimum Threshold value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Minimum Threshold
  @return void
 
*/
void UserConfigTask_UpdateTorqueMinimumThreshold(uint8_t value)
{
    //verify if value is in the range.
    if((value <= 100) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.torqueMinimumThreshold = value;
    }        
}

/**
  @brief Function to get Torque Sensor Multiplier
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Sensor Multiplier
  between 1 and 3.

NOTE: parameter passed in the PedalAssist_GetTorqueFromTS
*/
uint8_t UserConfigTask_GetTorqueSensorMultiplier(void)
{
    return userConfigData.PAS_ConfigData.torqueSensorMultiplier; 
}

/**
  @brief Function to update Torque Sensor Multiplier value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Sensor Multiplier
  @return void

*/
void UserConfigTask_UpdateTorqueSensorMultiplier(uint8_t value)
{
    //verify if value is in the range.
    if((value <= 3) && (value >= 1))
    {
        userConfigData.PAS_ConfigData.torqueSensorMultiplier = value; 
    }
}

/**
  @brief Function to get torque Max Speed
  read from data flash memory.
  
  @param void
  @return uint8_t !!!!range will be defined.

*/
uint8_t UserConfigTask_GetTorqueMaxSpeed(void)
{
    return userConfigData.PAS_ConfigData.torqueMaxSpeed; 
}

/**
  @brief Function to update torque Max Speed value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Max Speed
  @return void

*/
void UserConfigTask_UpdateTorqueMaxSpeed(uint8_t value)
{
    userConfigData.PAS_ConfigData.torqueMaxSpeed = value; 
}

/**
  @brief Function to get cadence Hybrid Leve Speed
  read from data flash memory.
  
  @param void
  @return uint8_t speed up to which this PAS level will give motor assistance
          range bewteen 0-40.

*/
uint8_t UserConfigTask_GetCadenceHybridLevelSpeed(uint8_t pasLevel)
{
    //
    if (pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.cadenceHybridLeveSpeed[pasLevel];
    }
    else
    {
        return userConfigData.PAS_ConfigData.cadenceHybridLeveSpeed[0];
    }   
}

/**
  @brief Function to update cadence Hybrid Leve Speed value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Cadence Hybrid Leve Speed
  @return void

*/
void UserConfigTask_UpdateCadenceHybridLeveSpeed(uint8_t pasLevel, uint8_t value)
{
    //
    if ((pasLevel < 10) && (value <= 40))
    {
        userConfigData.PAS_ConfigData.cadenceHybridLeveSpeed[pasLevel] = value;
    }
}

/**
  @brief Function to get torqueLevelPower
  read from data flash memory.
  
  @param void
  @return uint8_t Percentage of max motor assistance that this PAS level will give
          range bewteen 0-x.

*/
uint8_t UserConfigTask_GetTorqueLevelPower(uint8_t pasLevel)
{
    //
    if (pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.torqueLevelPower[pasLevel];
    }
    else
    {
        return userConfigData.PAS_ConfigData.torqueLevelPower[0];
    }   
}

/**
  @brief Function to update torqueLevelPower value
  read from data flash memory.
  
  @param uint8_t value to be passed into the torqueLevelPower
  @return void

*/
void UserConfigTask_UpdateTorqueLevelPower(uint8_t pasLevel, uint8_t value)
{
    //
    if ((pasLevel < 10) && (value <= 40))
    {
        userConfigData.PAS_ConfigData.torqueLevelPower[pasLevel] = value;
    }
 
}

/**
  @brief Function to get bike max speed
  read from data flash memory.
  
  @param void
  @return uint8_t The max speed that the throttle will bring the vehicle to.
          range bewteen 0-75.

*/
uint8_t UserConfigTask_GetBikeMaxSpeed(void)
{
    return userConfigData.Throttle_ConfigData.maxSpeed;
}

/**
  @brief Function to update bike max speed value
  read from data flash memory.
  
  @param uint8_t value to be passed into the bike max speed
  @return void

*/
void UserConfigTask_UpdateBikeMaxSpeed(uint8_t value)
{
    if((value <= 75) && (value >= 0))
    {
        userConfigData.Throttle_ConfigData.maxSpeed = value;
    }
}

/**
  @brief Function to get bike speed on walk mode
  read from data flash memory.
  
  @param void
  @return uint8_t Speed that the walk mode of the vehicle goes up to.
          range bewteen 0-10.

*/
uint8_t UserConfigTask_GetWalkModeSpeed(void)
{
    return userConfigData.Throttle_ConfigData.walkModeSpeed;
}

/**
  @brief Function to update bike speed on walk mode value
  read from data flash memory.
  
  @param uint8_t value to be passed into the walk mode speed
  @return void

*/
void UserConfigTask_UpdateWalkModeSpeed(uint8_t value)
{
    if((value <= 10) && (value >= 0))
    {
    userConfigData.Throttle_ConfigData.walkModeSpeed = value;
    }
}

/**
  @brief Function used to calculate a CRC 16 type using the same polynom 
  used by the bluetooth protocol.CCITT 16 bits polynom.
  
  @param uint8_t * data pointer to the data buffer where the CRC must be done.
  @param uint8_t lenght the lenght of the data to be calculated by the CRC algorithm.
  @return uint16_t return the calculated CRC.

*/
uint16_t UserConfigTask_CalculateCRC(uint8_t * buffer, uint8_t lenght)
{   
    uint8_t i;
    uint8_t n = 0;
    uint8_t value;
    uint16_t crc = 0x0000;
    //calculate the crc to all buffer lenght
    while(n < lenght)
    {
        //receive the next byte to be processed.
        value = buffer[n];
        
        //loop that implement the crc algorithm to each byte.
        for (i = 0; i < 8; i++) 
        {
            if (((crc & 0x8000) >> 8) ^ (value & 0x80))
            {
                crc = (uint16_t)(crc << 1) ^ CCITT_POLYNOM;
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }

            value <<= 1;
        }
        //increment the count.
        n++;
    }
  
	return crc;
}
