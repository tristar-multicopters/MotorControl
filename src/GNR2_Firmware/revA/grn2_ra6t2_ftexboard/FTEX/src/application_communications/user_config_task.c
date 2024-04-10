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
#include "user_config_data.h"
#pragma clang diagnostic pop

#include "user_config_task.h"
#include "motor_signal_processing.h"

#include "ASSERT_FTEX.h"
#include "wheel.h"

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
    .PAS_ConfigData.NumberOfPasLevels = PAS_MAX_LEVEL,
    .PAS_ConfigData.PasMaxTorqueRatio = PAS_MAX_TORQUE_RATIO,
    .PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupSpeed = PTS_OFFSET_STARTUP_SPEED_KMH,
    .PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupThreshold = PTS_OFFSET_PTS2TORQUE_STARTUP,
    .PAS_ConfigData.PAS_Startup_Detection.pasCadenceStartupNumbPulses = PEDALSPEEDSENSOR_MIN_PULSE_STARTUP,
    .PAS_ConfigData.PAS_Startup_Detection.pasCadenceStartupWindows = PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS,
    .PAS_ConfigData.PAS_Startup_Detection.PasAlgorithmStartup = PAS_DETECTIONSTARTUP_ALGORITHM,
    .PAS_ConfigData.PAS_Running_Detection.pasTorqueRunningThreshold = PTS_OFFSET_PTS2TORQUE,
    .PAS_ConfigData.PAS_Running_Detection.pasCadenceRunningNumbPulses = PEDALSPEEDSENSOR_MIN_PULSE_RUNNING,
    .PAS_ConfigData.PAS_Running_Detection.pasCadenceRunningWindows = PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS,
    .PAS_ConfigData.PAS_Running_Detection.PasAlgorithmRunning = PAS_DETECTIONRUNNING_ALGORITHM,
    .PAS_ConfigData.TorqueSensorMultiplier[0] = PAS_1_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[1] = PAS_2_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[2] = PAS_3_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[3] = PAS_4_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[4] = PAS_5_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[5] = PAS_6_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[6] = PAS_7_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[7] = PAS_8_TORQUE_GAIN,
    .PAS_ConfigData.TorqueSensorMultiplier[8] = PAS_9_TORQUE_GAIN,
    
    .PAS_ConfigData.PasLevelSpeed[0] = PAS_LEVEL_SPEED_1,
    .PAS_ConfigData.PasLevelSpeed[1] = PAS_LEVEL_SPEED_2,
    .PAS_ConfigData.PasLevelSpeed[2] = PAS_LEVEL_SPEED_3,
    .PAS_ConfigData.PasLevelSpeed[3] = PAS_LEVEL_SPEED_4,
    .PAS_ConfigData.PasLevelSpeed[4] = PAS_LEVEL_SPEED_5,
    .PAS_ConfigData.PasLevelSpeed[5] = PAS_LEVEL_SPEED_6,
    .PAS_ConfigData.PasLevelSpeed[6] = PAS_LEVEL_SPEED_7,
    .PAS_ConfigData.PasLevelSpeed[7] = PAS_LEVEL_SPEED_8,
    .PAS_ConfigData.PasLevelSpeed[8] = PAS_LEVEL_SPEED_9,
    
    .PAS_ConfigData.PasLevelMinTorque[0] = PAS_1_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[1] = PAS_2_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[2] = PAS_3_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[3] = PAS_4_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[4] = PAS_5_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[5] = PAS_6_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[6] = PAS_7_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[7] = PAS_8_MIN_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMinTorque[8] = PAS_9_MIN_TORQUE_PERCENT,
    
    .PAS_ConfigData.PasLevelMaxTorque[0] = PAS_1_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[1] = PAS_2_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[2] = PAS_3_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[3] = PAS_4_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[4] = PAS_5_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[5] = PAS_6_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[6] = PAS_7_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[7] = PAS_8_MAX_TORQUE_PERCENT,
    .PAS_ConfigData.PasLevelMaxTorque[8] = PAS_9_MAX_TORQUE_PERCENT,
    
    .PAS_ConfigData.PasAccelRampType[0] = PAS1_ACCEL_RAMP_TYPE, 
    .PAS_ConfigData.PasAccelRampType[1] = PAS2_ACCEL_RAMP_TYPE, 
    .PAS_ConfigData.PasAccelRampType[2] = PAS3_ACCEL_RAMP_TYPE, 
    .PAS_ConfigData.PasAccelRampType[3] = PAS4_ACCEL_RAMP_TYPE, 
    .PAS_ConfigData.PasAccelRampType[4] = PAS5_ACCEL_RAMP_TYPE, 
    .PAS_ConfigData.PasAccelRampType[5] = PAS6_ACCEL_RAMP_TYPE, 
    .PAS_ConfigData.PasAccelRampType[6] = PAS7_ACCEL_RAMP_TYPE,
    .PAS_ConfigData.PasAccelRampType[7] = PAS8_ACCEL_RAMP_TYPE,
    .PAS_ConfigData.PasAccelRampType[8] = PAS9_ACCEL_RAMP_TYPE,
    
    .PAS_ConfigData.PasAccelRampArg1[0] = PAS1_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[1] = PAS2_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[2] = PAS3_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[3] = PAS4_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[4] = PAS5_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[5] = PAS6_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[6] = PAS7_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[7] = PAS8_ACCEL_RAMP_ARG1,
    .PAS_ConfigData.PasAccelRampArg1[8] = PAS9_ACCEL_RAMP_ARG1,
    
    .PAS_ConfigData.PasDecelRampType[0] = PAS1_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[1] = PAS2_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[2] = PAS3_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[3] = PAS4_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[4] = PAS5_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[5] = PAS6_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[6] = PAS7_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[7] = PAS8_DECEL_RAMP_TYPE,
    .PAS_ConfigData.PasDecelRampType[8] = PAS9_DECEL_RAMP_TYPE,
    
    .PAS_ConfigData.PasDecelRampArg1[0] = PAS1_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[1] = PAS2_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[2] = PAS3_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[3] = PAS4_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[4] = PAS5_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[5] = PAS6_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[6] = PAS7_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[7] = PAS8_DECEL_RAMP_ARG1,
    .PAS_ConfigData.PasDecelRampArg1[8] = PAS9_DECEL_RAMP_ARG1,   
    
    .PAS_ConfigData.PasSensorConfig.pasNbMagnetsPerTurn = 0, // To update
    .PAS_ConfigData.PasSensorConfig.pasTorqueInputMax = PTS_MAX_PTSVALUE,
    .PAS_ConfigData.PasSensorConfig.pasTorqueInputMin = PTS_OFFSET_ADC2PTS,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[0] = PTS_SPEED_FILTER_1,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[1] = PTS_SPEED_FILTER_2,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW1[0] = PTS_FILTER_BW1_1,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW2[0] = PTS_FILTER_BW2_1,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW1[1] = PTS_FILTER_BW1_2,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW2[1] = PTS_FILTER_BW2_2,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW1[2] = PTS_FILTER_BW1_3,
    .PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW2[2] = PTS_FILTER_BW2_3,
    .Screen_ConfigData.Throttle_ConfigData.AdcOffset = THROTTLE_OFFSET_ADC2THROTTLE,
    .Screen_ConfigData.Throttle_ConfigData.AdcMax = THROTTLE_MAX_ADC2THROTTLE,
    .Battery_ConfigData.FullVoltage = BATTERY_FULL_VOLT_X_100,
    .Battery_ConfigData.EmptyVoltage = BATTERY_EMPTY_VOLT_X_100,   
    .Battery_ConfigData.MaxPeakDCCurrent = MAX_APPLICATION_CURRENT,
    .Battery_ConfigData.ContinuousDCCurrent = MAX_BMS_CONTINOUS_CURRENT,
    .Battery_ConfigData.PeakCurrentDeratingDuration =  (MAX_TIME_BMS_TOLERANT - MAX_POWER_LIMIT_TIMEOUT),
    .Battery_ConfigData.PeakCurrentMaxDuration = MAX_POWER_LIMIT_TIMEOUT,
    .Screen_ConfigData.WalkmodeSpeed = PAS_LEVEL_SPEED_WALK,
    .Screen_ConfigData.WalkmodeMaxTorque = PAS_WALK_POWER_PERCENT,
    .Screen_ConfigData.WalkmodeAccelRampType = WALKMODE_ACCEL_RAMP_TYPE,
    .Screen_ConfigData.WalkmodeAccelRampArg1 = WALKMODE_ACCEL_RAMP_ARG1,
    .Screen_ConfigData.MaxSpeed = VEHICLE_TOP_SPEED_KMH,
    .Screen_ConfigData.WheelDiameter = WHEEL_DIAMETER,
    .Screen_ConfigData.ScreenProtocol = SCREEN_PROTOCOL,
    .Screen_ConfigData.HeadLightDefault = POWERTRAIN_HEADLIGHT_DEFAULT,
    .Screen_ConfigData.TailLightDefault = POWERTRAIN_TAILLIGHT_DEFAULT,
    .Screen_ConfigData.TailLightBlinkOnBrake = REAR_LIGHT_BLINK_ON_BRAKE,
    .crc = 0x0000,
    .Screen_ConfigData.Throttle_ConfigData.ThrottleBlock = THROTTLE_BLOCK_OFF,
    .Screen_ConfigData.Throttle_ConfigData.MaxSpeed = VEHICLE_TOP_SPEED_KMH,
    .Screen_ConfigData.Throttle_ConfigData.AccelRampType = THROTTLE_ACCEL_RAMP_TYPE,
    .Screen_ConfigData.Throttle_ConfigData.AccelRampArg1 = THROTTLE_ACCEL_RAMP_ARG1, 
    .Screen_ConfigData.Motor_Signal_Parameters.motorMixedSignal = MOTOR_TEMP_MIXED,
    .Screen_ConfigData.Motor_Signal_Parameters.minSignalThreshold = MINIMUM_SIGNAL_THRESHOLD,
    .Screen_ConfigData.Motor_Signal_Parameters.maxWheelSpeedPeriodUs = MAX_WHEELSPEED_PERIOD_US,
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
    Data flash memory used to save data user configuration starts
    at address FLASH_HP_DF_BLOCK_4.
    Each block address has 64 bytes.
    The number of blocks used to hold the user configuration is defined
    here: NUMBER_OF_BLOCKS_USED.
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
        uCAL_Data_Flash_Read(userConfigHandle->pDataFlash_Handle, data, FLASH_HP_DF_BLOCK_4, USER_DATA_CONFIG_LENGTH);
        
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
                //erase seven blocks(block 4 unitl 10) of the data flash memory before write on it.
                //if you need more than 64 to write user configuration we need to
                //erase more than 1 block.
                uCAL_Data_Flash_Erase(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_4, NUMBER_OF_BLOCKS_USED);
            
                //check if all bytes were erased.
                //we are using 7 blocks.
                //to check 2 or more blocks use n*NUMBER_OF_BYTES_IN_THE_BLOCK.
                if (uCAL_Data_Flash_Blank_Check(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_4, NUMBER_OF_BLOCKS_USED*NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
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
                uCAL_Data_Flash_Write(userConfigHandle->pDataFlash_Handle,data,FLASH_HP_DF_BLOCK_4, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN); 
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
            
            //erase 7 blocks(block 4 until 10) of the data flash memory before write on it.
            //if you need more than 64 to write user configuration we need to
            //erase more than 1 block.
            uCAL_Data_Flash_Erase(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_4, NUMBER_OF_BLOCKS_USED);
            
            //check if all bytes were erased.
            //we are using just 1 block, so just 64 are checked.
            //to check 2 or more blocks use n*NUMBER_OF_BYTES_IN_THE_BLOCK.
            if (uCAL_Data_Flash_Blank_Check(userConfigHandle->pDataFlash_Handle, FLASH_HP_DF_BLOCK_4, NUMBER_OF_BLOCKS_USED*NUMBER_OF_BYTES_IN_THE_BLOCK) == true)
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
            uCAL_Data_Flash_Write(userConfigHandle->pDataFlash_Handle,data,FLASH_HP_DF_BLOCK_4, NUMBER_OF_BYTES_MULT_4_TO_BE_WRITTEN);                
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
    
    PWRT_Handle_t * paPowertrain = userConfigHandle->pVController->pPowertrain;
    
    //update userConfigData.PAS_ConfigData.numberOfPasLevels(PAS_MAX_LEVEL)
    paPowertrain->pPAS->sParameters.bMaxLevel = UserConfigTask_GetNumberPasLevels();
    
    //update userConfigData.PAS_ConfigData.pasMaxTorqueRatio(PAS_MAX_TORQUE_RATIO)
    paPowertrain->pPAS->sParameters.hMaxTorqueRatio = UserConfigTask_GetPasMaxTorqueRatio();
    
    //update userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupSpeed(PTS_OFFSET_STARTUP_SPEED_KMH)
    paPowertrain->pPAS->pPTS->hParameters.hStartupOffsetMTSpeedKMH = (uint16_t)UserConfigTask_GetPasTorqueStartupSpeed();

    //update userConfigData.PAS_ConfigData.pasTorqueStartupThreshold(PTS_OFFSET_PTS2TORQUE_STARTUP)
    paPowertrain->pPAS->pPTS->hParameters.hOffsetMTStartup = UserConfigTask_GetPasTorqueStartupThreshold();
    
    //paPowertrain->pPAS->pPSS->hPedalSpeedSens_MinPulseStartup(PEDALSPEEDSENSOR_MIN_PULSE_STARTUP)
    paPowertrain->pPAS->pPSS->hPedalSpeedSens_MinPulseStartup = UserConfigTask_GetPasCadenceStartupNumbPulses();
    
    //update userConfigHandle->pVController->pPowertrain->pPAS->pPSS->wPedalSpeedSens_WindowsStartup(PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS)
    paPowertrain->pPAS->pPSS->wPedalSpeedSens_WindowsStartup = UserConfigTask_GetPasCadenceStartupWindows();
    
    //passe to the system the pas detection algorithm on startup condition
    paPowertrain->pPAS->bStartupPasAlgorithm = UserConfigTask_GetPasAlgorithmStartup();
    
     //update userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasTorqueRunningThreshold(PTS_OFFSET_PTS2TORQUE)
    paPowertrain->pPAS->pPTS->hParameters.hOffsetMT = UserConfigTask_GetPasTorqueRunningThreshold();
    
    //paPowertrain->pPAS->pPSS->hPedalSpeedSens_MinPulseRunning(PEDALSPEEDSENSOR_MIN_PULSE_RUNNING)
    paPowertrain->pPAS->pPSS->hPedalSpeedSens_MinPulseRunning = UserConfigTask_GetPasCadenceRunningNumbPulses();
    
    //update userConfigHandle->pVController->pPowertrain->pPAS->pPSS->wPedalSpeedSens_WindowsRunning(PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS)
    paPowertrain->pPAS->pPSS->wPedalSpeedSens_WindowsRunning = UserConfigTask_GetPasCadenceRunningWindows();
    
    //passe to the system the pas detection algorithm on running condition
    paPowertrain->pPAS->bRunningPasAlgorithm = UserConfigTask_GetPasAlgorithmRunning();
    
      
    for(uint8_t n = PAS_1;n <= PAS_9;n++)
    {
        //update PAS_ConfigData.torqueSensorMultiplier(PAS_TORQUE_GAIN) 
        paPowertrain->pPAS->sParameters.bTorqueGain[n] = UserConfigTask_GetTorqueSensorMultiplier(n);
        //update PASMaxSpeed, cadence speed by PAS level. 
        paPowertrain->pPAS->sParameters.PASMaxSpeed[n] = UserConfigTask_GetPasLevelSpeed(n);
        //
        paPowertrain->pPAS->sParameters.PASMinTorqRatiosInPercentage[n] = UserConfigTask_GetPasLevelMinTorque(n);
        //
        paPowertrain->pPAS->sParameters.PASMaxTorqRatiosInPercentage[n] = UserConfigTask_GetPasLevelMaxTorque(n);
        
        paPowertrain->pPAS->sParameters.PasRamps[0][n].RampType = UserConfigTask_GetPasAccelRampType(n);
        paPowertrain->pPAS->sParameters.PasRamps[1][n].RampType = UserConfigTask_GetPasDecelRampType(n);
        
        paPowertrain->pPAS->sParameters.PasRamps[0][n].LinearParameters.Alpha = UserConfigTask_GetPasAccelRampArg1(n);
        paPowertrain->pPAS->sParameters.PasRamps[1][n].LinearParameters.Alpha = UserConfigTask_GetPasDecelRampArg1(n);
    }
    
    paPowertrain->pPAS->sParameters.walkModeTorqueRatio = UserConfigTask_GetWalkmodeMaxTorque();
    paPowertrain->pPAS->sParameters.PasWalkmodeRamp.RampType = UserConfigTask_GetWalkmodeAccelRampType();
    paPowertrain->pPAS->sParameters.PasWalkmodeRamp.LinearParameters.Alpha =  UserConfigTask_GetWalkmodeAccelRampArg1();

    
    //update vehicle max speed(VEHICLE_TOP_SPEED_KMH).
    paPowertrain->sParameters.VehicleMaxSpeed = UserConfigTask_GetBikeMaxSpeed();

    //Magnets per pedal turn doesn't exist in vc layer yet so there is nothing to initialise
  
    paPowertrain->pPAS->pPTS->hParameters.hOffsetPTS = UserConfigTask_GetPasTorqueInputMin();
    paPowertrain->pPAS->pPTS->hParameters.hMax       = UserConfigTask_GetPasTorqueInputMax();
    
    //Throttle_ConfigData.walkMOdeSpeed(PAS_LEVEL_SPEED_WALK) is not passed
    //directly to any variable. Because of this is not updated here.
    
    
    Wheel_SetWheelDiameter(UserConfigTask_GetWheelDiameter()); 
    
    
    if(SCREEN_PROTOCOL != UART_LOG_HS) // This prevents the user config from blocking the use of high speed log 
    {                                  // which can only be set by changing the define and building the code   
        UART0Handle.UARTProtocol =  UserConfigTask_GetScreenProtocol();
    }

    paPowertrain->pHeadLight->bDefaultLightState  =  UserConfigTask_GetHeadLightDefault(); 

    paPowertrain->pTailLight->bDefaultLightState  =  UserConfigTask_GetTailLightDefault();
    userConfigHandle->pVController->pPowertrain->pTailLight->bBlinkOnBrake       =  UserConfigTask_GetTailLightBlinkOnBrake();

    paPowertrain->pThrottle->hParameters.hOffsetThrottle = UserConfigTask_GetThrottleAdcOffset();
    paPowertrain->pThrottle->hParameters.hMaxThrottle    = UserConfigTask_GetThrottleAdcMax();  
    paPowertrain->pThrottle->BlockOffThrottle = UserConfigTask_GetThrottleBlockOff();
    paPowertrain->pThrottle->hParameters.DefaultMaxThrottleSpeedKMH= UserConfigTask_GetThrottleMaxSpeed();
    
    paPowertrain->pThrottle->hParameters.ThrottleRamps[0].RampType = UserConfigTask_GetThrottleAccelRampType();
    paPowertrain->pThrottle->hParameters.ThrottleRamps[0].LinearParameters.Alpha = UserConfigTask_GetThrottleAccelRampArg1();

    
    paPowertrain->pBatMonitorHandle->VBatMax = UserConfigTask_GetBatteryFullVoltage();
    paPowertrain->pBatMonitorHandle->VBatMin = UserConfigTask_GetBatteryEmptyVoltage();
    
    paPowertrain->pMDI->pMCI->pSpeedTorqCtrl->hMaxBusCurrent = UserConfigTask_GetBatteryMaxPeakDCCurrent();

    paPowertrain->pMDI->pMCI->pSpeedTorqCtrl->hMaxContinuousCurrent = UserConfigTask_GetBatteryContinuousDCCurrent();

    paPowertrain->pMDI->pMCI->pSpeedTorqCtrl->FoldbackDynamicMaxPower.hDecreasingEndValue  = UserConfigTask_GetBatteryPeakCurrentMaxDuration() + UserConfigTask_GetBatteryPeakCurrentDeratingDuration();
    
    paPowertrain->pMDI->pMCI->pSpeedTorqCtrl->FoldbackDynamicMaxPower.hDecreasingRange = UserConfigTask_GetBatteryPeakCurrentDeratingDuration();

    //
    for (uint8_t n = 0; n < FILTERSPEED_ARRAY_SIZE; n++)
    {
        paPowertrain->pPAS->pPTS->hParameters.hFilterSpeed[n] = UserConfigTask_GetFilterSpeed(n);
    }
    
    //
    for (uint8_t n = 0; n < BW_ARRAY_SIZE; n++)
    {
        paPowertrain->pPAS->pPTS->hParameters.hLowPassFilterBW1[n] = UserConfigTask_GetFilterBwValue(n,BW1);
        paPowertrain->pPAS->pPTS->hParameters.hLowPassFilterBW2[n] = UserConfigTask_GetFilterBwValue(n,BW2);
    }
    
    updateisMotorMixedSignalValue(UserConfigTask_GetMotorMixedSignalState());
    updateMinSignalThresholdValue(UserConfigTask_GetMinSignalThreshold());
    updateMaxWheelSpeedPeriodUsValue(UserConfigTask_GetMaxWheelSpeedPeriodUs());
    
}


/**
  @brief Function to get number Of Pas Levels
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent the number Of Pas Levels.
*/
uint8_t UserConfigTask_GetNumberPasLevels(void)
{
    return userConfigData.PAS_ConfigData.NumberOfPasLevels; 
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
    if((value < 10) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.NumberOfPasLevels = value;
    }        
}

/**
  @brief Function to get Pas Max Power
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Pas Max Torque Ratio
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetPasMaxTorqueRatio(void)
{
    return userConfigData.PAS_ConfigData.PasMaxTorqueRatio; 
}

/**
  @brief Function to update Pas Max Torque Ratio value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Pas Max Power
  @return  void
*/
void UserConfigTask_UpdatePasMaxTorqueRatio(uint8_t value)
{
    //verify if vakue is in the range.
    if((value <= 100) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.PasMaxTorqueRatio = value;    
    }
}

/**
  @brief Function to get Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetPasTorqueStartupSpeed(void)
{
    return userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupSpeed;     
}

/**
  @brief Function to update Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Minimum Threshold Startup
  @return void
 
*/
void UserConfigTask_UpdatePasTorqueStartupSpeed(uint8_t value)
{
    //verify if value is in the range.
    if((value <= 100) && (value > 0))
    {
        userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupSpeed = value;
    }        
}

/**
  @brief Function to get Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Startup Offset Minimum Threshold Speed
  in RPM.
*/
uint8_t UserConfigTask_GetPasTorqueStartupThreshold(void)
{
    return userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupThreshold; 
}

/**
  @brief Function to update Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param uint8_t value to be passed into the Startup Offset Minimum Threshold Speed
  @return void
 
*/
void UserConfigTask_UpdatePasTorqueStartupThreshold(uint8_t value)
{
    //verify if value is in the range.
    if((value <= 100) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasTorqueStartupThreshold = value;
    }        
}

/**
  @brief Function to get pasCadenceStartupNumbPulses
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent pasCadenceStartupNumbPulses.
*/
uint8_t UserConfigTask_GetPasCadenceStartupNumbPulses(void)
{
    return userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasCadenceStartupNumbPulses; 
}

/**
  @brief Function to update pasCadenceStartupNumbPulses
  read from data flash memory.
  
  @param uint8_t value to be passed into the pasCadenceStartupNumbPulses
  @return void
 
*/
void UserConfigTask_UpdatePasCadenceStartupNumbPulses(uint8_t value)
{
    //verify if value is in the range.
    if((value > 0) && (value <= 0xFF))
    {
        userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasCadenceStartupNumbPulses = value;
    }        
}

/**
  @brief Function to get time windows used to check the number of
  pulses detected from the cadence sensor read from data flash memory.
  
  @param void
  @return uint16_t a number that represent time windows used to check the number of
  pulses.
*/
uint16_t UserConfigTask_GetPasCadenceStartupWindows(void)
{
    return userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasCadenceStartupWindows;
}

/**
  @brief Function to update time windows used to check the number of
  pulses detected from the cadence sensor read from data flash memory.
  
  @param uint16_t value to be passed into the time windows used to check the number of
  pulses.
  @return void
 
*/
void UserConfigTask_UpdatePasCadenceStartupWindows(uint16_t value)
{
    //verify if value is in the range.
    //range on ms.
    if((value >= 200) && (value <= 0xFFFF))
    {
        userConfigData.PAS_ConfigData.PAS_Startup_Detection.pasCadenceStartupWindows = value;
    }   
}

/**
  @brief Function to get pas algorithm detection used on startup state
  read from data flash memory.
  
  @param void
  @return uint23_t pas algorithm detection used on startup state.
*/
uint8_t UserConfigTask_GetPasAlgorithmStartup(void)
{
    return userConfigData.PAS_ConfigData.PAS_Startup_Detection.PasAlgorithmStartup;
}

/**
  @brief Function to update pas algorithm detection used on startup state
  read from data flash memory.
  
  @param uint32_t value equivalent to the pas algorithm detection that must be used.
  @return void
 
*/
void UserConfigTask_UpdatePasPasAlgorithmStartup(uint8_t value)
{
    //verify if value is in the range.
    //HybridOrSensorUse is the maximum vale, on this case 4.
    if((value >= 0) && (value <= MAX_PASALGORITHM_VALUE))
    {
        userConfigData.PAS_ConfigData.PAS_Startup_Detection.PasAlgorithmStartup = value;
    }   
}

/**
  @brief Function to get Torque Sensor Multiplier(GAIN)by PAS level
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Sensor Multiplier
  between 1 and 3.

NOTE: parameter passed in the PedalAssist_GetTorqueFromTS
*/
uint16_t UserConfigTask_GetTorqueSensorMultiplier(uint8_t pasLevel)
{
    //verify if value is in the range.
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.TorqueSensorMultiplier[pasLevel-1];
    }
    else
    {
        return 0;
    }    
}

/**
  @brief Function to update Torque Sensor Multiplier value(GAIN) by PAS level
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Sensor Multiplier
  @return void

*/
void UserConfigTask_UpdateTorqueSensorMultiplier(uint8_t pasLevel, uint16_t value)
{
    //
    if ((0 < pasLevel && pasLevel < 10) && (value <= 0xFFFF))
    {
        userConfigData.PAS_ConfigData.TorqueSensorMultiplier[pasLevel-1] = value;
    }
}

/**
  @brief Function to get torque Max Speed
  read from data flash memory.
  
  @param void
  @return uint8_t !!!!range will be defined.

*/
uint8_t UserConfigTask_GetPasLevelMinTorque(uint8_t pasLevel)
{
    //
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.PasLevelMinTorque[pasLevel-1];
    }
    else
    {
        return 0;
    }    
}

/**
  @brief Function to update torque Max Speed value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Max Speed
  @return void

*/
void UserConfigTask_UpdatePasLevelMinTorque( uint8_t pasLevel, uint8_t value)
{
    //
    if ((0 < pasLevel && pasLevel < 10) && (value <= 100))
    {
        userConfigData.PAS_ConfigData.PasLevelMinTorque[pasLevel-1] = value;
    }
}

/**
  @brief Function to get speed to the current Pas Level
  read from data flash memory.
  
  @param void
  @return uint8_t speed up to which this PAS level will give motor assistance
          range bewteen 0-40.

*/
uint8_t UserConfigTask_GetPasLevelSpeed(uint8_t pasLevel)
{
    //
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.PasLevelSpeed[pasLevel-1];
    }
    else
    {
        return 0;
    }   
}

/**
  @brief Function to update speed to the current Pas Level
  read from data flash memory.
  
  @param uint8_t value to be passed into the Cadence Level Speed
  @return void

*/
void UserConfigTask_UpdatePasLevelSpeed(uint8_t pasLevel, uint8_t value)
{
    //
    if ((0 < pasLevel && pasLevel < 10) && (value <= 32))
    {
        userConfigData.PAS_ConfigData.PasLevelSpeed[pasLevel-1] = value;
    }
}

/**
  @brief Function to get pasLevelMaxTorque
  read from data flash memory.
  
  @param void
  @return uint8_t Percentage of max motor assistance that this PAS level will give
          range bewteen 0-x.

*/
uint8_t UserConfigTask_GetPasLevelMaxTorque(uint8_t pasLevel)
{
    // 
    if (0 < pasLevel && pasLevel < 10)
    {        
        return userConfigData.PAS_ConfigData.PasLevelMaxTorque[pasLevel-1];
    }
    else
    {
        return 0;
    }   
}

/**
  @brief Function to update pasLevelMaxTorque value
  read from data flash memory.
  
  @param uint8_t value to be passed into the pasLevelMaxTorque
  @return void

*/
void UserConfigTask_UpdatePasLevelMaxTorque(uint8_t pasLevel, uint8_t value)
{
    //
    if ((0 < pasLevel && pasLevel < 10) && (value <= 100))
    {
        userConfigData.PAS_ConfigData.PasLevelMaxTorque[pasLevel-1] = value;
    }
}

/**
  @brief Function to get Pas Nb Magnets Per Turn
  read from data flash memory.
  
  @param void
  @return uint8_t Number of magnets per pedal rotation, cannot be 0

*/
uint8_t UserConfigTask_GetPasNbMagnetsPerTurn(void)
{
    return userConfigData.PAS_ConfigData.PasSensorConfig.pasNbMagnetsPerTurn;
}

/**
  @brief Function to update Pas Nb Magnets Per Turn value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Pas Nb Magnets Per Turn cannot be 0
  @return void

*/
void UserConfigTask_UpdatePasNbMagnetsPerTurn(uint8_t value)
{
    if (value > 0)
    {
        userConfigData.PAS_ConfigData.PasSensorConfig.pasNbMagnetsPerTurn = value;
    }
}

/**
  @brief Function to get Pas Torque Input Max
  read from data flash memory.
  
  @param void
  @return uint16_t Pas Torque Input Max

*/
uint16_t UserConfigTask_GetPasTorqueInputMax(void)
{
    return userConfigData.PAS_ConfigData.PasSensorConfig.pasTorqueInputMax;
}

/**
  @brief Function to update Pas Torque Input Max value
  read from data flash memory.
  
  @param uint16_t value to be passed into the Pas Torque Input Max value
  @return void

*/
void UserConfigTask_UpdatePasTorqueInputMax(uint16_t value)
{
    if (value > userConfigData.PAS_ConfigData.PasSensorConfig.pasTorqueInputMin)
    {    
        if (value < DIGITAL33_0_7_VOLTS)
        {
            value = DIGITAL33_0_7_VOLTS;
        }
    
        userConfigData.PAS_ConfigData.PasSensorConfig.pasTorqueInputMax = value;   
    }
}

/**
  @brief Function to get Pas Torque Input Min
  read from data flash memory.
  
  @param void
  @return uint16_t Pas Torque Input Min

*/
uint16_t UserConfigTask_GetPasTorqueInputMin(void)
{
    return userConfigData.PAS_ConfigData.PasSensorConfig.pasTorqueInputMin;  
}

/**
  @brief Function to update Pas Torque Input Min
  read from data flash memory.
  
  @param uint16_t value to be passed into the Pas Torque Input Min
  @return void

*/
void UserConfigTask_UpdatePasTorqueInputMin(uint16_t value)
{
    if (value < userConfigData.PAS_ConfigData.PasSensorConfig.pasTorqueInputMax)
    {    
        if (value < DIGITAL33_0_7_VOLTS)
        {
            value = DIGITAL33_0_7_VOLTS;
        }
    
        userConfigData.PAS_ConfigData.PasSensorConfig.pasTorqueInputMin = value;   
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
    return userConfigData.Screen_ConfigData.MaxSpeed;
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
        userConfigData.Screen_ConfigData.MaxSpeed = value;
    }
}

/**
  @brief Function to get bike speed on walk mode
  read from data flash memory.
  
  @param void
  @return uint8_t Speed that the walk mode of the vehicle goes up to.
          range bewteen 0-10.

*/
uint8_t UserConfigTask_GetWalkmodeSpeed(void)
{
    return userConfigData.Screen_ConfigData.WalkmodeSpeed;
}

/**
  @brief Function to update bike speed on walk mode value
  read from data flash memory.
  
  @param uint8_t value to be passed into the walk mode speed
  @return void

*/
void UserConfigTask_UpdateWalkmodeSpeed(uint8_t value)
{
    if ((value <= 10) && (value >= 0))
    {
        userConfigData.Screen_ConfigData.WalkmodeSpeed = value;
    }
}

/**
  @brief Function to get max torque on walk mode
  read from data flash memory.
  
  @param void
  @return uint8_t max torque in % that the walk mode of the vehicle goes up to.

*/
uint8_t UserConfigTask_GetWalkmodeMaxTorque(void)
{
    return userConfigData.Screen_ConfigData.WalkmodeMaxTorque;
}

/**
  @brief Function to update max torque on walk mode value
  read from data flash memory.
  
  @param uint8_t value to be passed into the walk mode max torque in %
  @return void

*/
void UserConfigTask_UpdateWalkmodeMaxTorque(uint8_t value)
{
    if (value <= 100)
    {
       userConfigData.Screen_ConfigData.WalkmodeMaxTorque = value;
    }
    else
    {
       userConfigData.Screen_ConfigData.WalkmodeMaxTorque = 100;
    }        
}

/**
  @brief Function to get walkmode acceleration ramp type read from data flash memory.
  
  @param  void
  @return uint8_t type of ramp.
 
*/
uint8_t UserConfigTask_GetWalkmodeAccelRampType(void)
{
    return userConfigData.Screen_ConfigData.WalkmodeAccelRampType;    
}

/**
  @brief Function to update the walkmode acceleration ramp type read from data flash memory.
  
  @param uint8_t type of ramp.
  @return void
 
*/
void UserConfigTask_UpdateWalkmodeAccelRampType(uint8_t rampType)
{
    userConfigData.Screen_ConfigData.WalkmodeAccelRampType = rampType;
}

/**
  @brief Function to get walkmode acceleration ramp argument 1 read from data flash memory.
  
  @param  void
  @return uint16_t argument 1
 
*/
uint16_t UserConfigTask_GetWalkmodeAccelRampArg1(void)
{
    return userConfigData.Screen_ConfigData.WalkmodeAccelRampArg1;      
}

/**
  @brief Function to update the walkmode acceleration ramp argument 1 read from data flash memory.
  
  @param uint16_t argument 1.
  @return void
 
*/
void UserConfigTask_UpdateWalkmodeAccelRampArg1(uint16_t arg1)
{
    userConfigData.Screen_ConfigData.WalkmodeAccelRampArg1 = arg1; 
}

/**
  @brief Function to get Wheel Diameter
  read from data flash memory.
  
  @param void
  @return uint8_t Wheel Diameter

*/
uint8_t UserConfigTask_GetWheelDiameter(void)
{
    return userConfigData.Screen_ConfigData.WheelDiameter;
}

/**
  @brief Function to update bike Wheel Diameter value
  read from data flash memory.
  
  @param uint8_t value to be passed into the WheelDiameter
  @return void

*/
void UserConfigTask_UpdateWheelDiameter(uint8_t value)
{
    if ((value <= 50) && (value >= 0))
    {
        userConfigData.Screen_ConfigData.WheelDiameter = value;
    }
}

/**
  @brief Function to get Screen Protocol
  read from data flash memory.
  
  @param void
  @return uint8_t Screen Protocol

*/
uint8_t UserConfigTask_GetScreenProtocol(void)
{
    return userConfigData.Screen_ConfigData.ScreenProtocol;
}

/**
  @brief Function to update bike Screen Protocol value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Screen Protocol
  @return void

*/
void UserConfigTask_UpdateScreenProtocol(uint8_t value)
{
    if (value == UART_APT || value == UART_CLOUD_5S || value == UART_DISABLE)
    {
        userConfigData.Screen_ConfigData.ScreenProtocol = value;
    }
}

/**
  @brief Function to get HeadLight Default
  read from data flash memory.
  
  @param void
  @return uint8_t HeadLight Default

*/
uint8_t UserConfigTask_GetHeadLightDefault(void)
{
    return userConfigData.Screen_ConfigData.HeadLightDefault;
}

/**
  @brief Function to update bike HeadLight Default value
  read from data flash memory.
  
  @param uint8_t value to be passed into the HeadLight Default
  @return void

*/
void UserConfigTask_UpdateHeadLightDefault(uint8_t value)
{
    if (value > 0)
    {
        userConfigData.Screen_ConfigData.HeadLightDefault = 1;
    }
    else
    {
        userConfigData.Screen_ConfigData.HeadLightDefault = 0;    
    }
}

/**
  @brief Function to get TailLight Default
  read from data flash memory.
  
  @param void
  @return uint8_t TailLight Default

*/
uint8_t UserConfigTask_GetTailLightDefault(void)
{
    return userConfigData.Screen_ConfigData.TailLightDefault;
}

/**
  @brief Function to update bike TailLight Default value
  read from data flash memory.
  
  @param uint8_t value to be passed into the TailLight Default
  @return void

*/
void UserConfigTask_UpdateTailLightDefault(uint8_t value)
{
    if (value > 0)
    {
        userConfigData.Screen_ConfigData.TailLightDefault = 1;
    }
    else
    {
        userConfigData.Screen_ConfigData.TailLightDefault = 0;    
    }
}

/**
  @brief Function to get TailLight Blink On Brake
  read from data flash memory.
  
  @param void
  @return uint8_t TailLight Blink On Brake

*/
uint8_t UserConfigTask_GetTailLightBlinkOnBrake(void)
{
    return userConfigData.Screen_ConfigData.TailLightBlinkOnBrake;
}

/**
  @brief Function to update bike TailLight Blink On Brake value
  read from data flash memory.
  
  @param uint8_t value to be passed into the TailLight Blink On Brake
  @return void

*/
void UserConfigTask_UpdateTailLightBlinkOnBrake(uint8_t value)
{
    if (value > 0)
    {
        userConfigData.Screen_ConfigData.TailLightBlinkOnBrake = 1;
    }
    else
    {
        userConfigData.Screen_ConfigData.TailLightBlinkOnBrake = 0;    
    }
}

/**
  @brief Function to get Throttle Adc Offset
  read from data flash memory.
  
  @param void
  @return uint16_t Throttle Adc Offset

*/
uint16_t UserConfigTask_GetThrottleAdcOffset(void)
{
    return userConfigData.Screen_ConfigData.Throttle_ConfigData.AdcOffset;
}

/**
  @brief Function to update Throttle Adc Offset
  read from data flash memory.
  
  @param uint16_t value to be passed into the Throttle Adc Offset
  @return void

*/
void UserConfigTask_UpdateThrottleAdcOffset(uint16_t value)
{
    if(value <= userConfigData.Screen_ConfigData.Throttle_ConfigData.AdcMax)
    {
        if(value < DIGITAL5_0_8_VOLTS)
        {
           value = DIGITAL5_0_8_VOLTS;  
        }   
        
        userConfigData.Screen_ConfigData.Throttle_ConfigData.AdcOffset = value;
    }   
}

/**
  @brief Function to get Throttle Adc Max
  read from data flash memory.
  
  @param void
  @return uint16_t Throttle Adc Max

*/
uint16_t UserConfigTask_GetThrottleAdcMax(void)
{
    return userConfigData.Screen_ConfigData.Throttle_ConfigData.AdcMax;
}

/**
  @brief Function to update Throttle Adc Max
  read from data flash memory.
  
  @param uint16_t value to be passed into the Throttle Adc Max
  @return void

*/
void UserConfigTask_UpdateThrottleAdcMax(uint16_t value)
{       
    if(value >= userConfigData.Screen_ConfigData.Throttle_ConfigData.AdcOffset)
    {
        if(value < DIGITAL5_0_8_VOLTS)
        {
           value = DIGITAL5_0_8_VOLTS;  
        } 
        
        userConfigData.Screen_ConfigData.Throttle_ConfigData.AdcMax = (uint16_t)value;
    }
}

/**
  @brief Function to get throttle acceleration ramp type read from data flash memory.
  
  @param  void
  @return uint8_t type of ramp.
 
*/
uint8_t UserConfigTask_GetThrottleAccelRampType(void)
{
    return userConfigData.Screen_ConfigData.Throttle_ConfigData.AccelRampType;    
}

/**
  @brief Function to update the throttle acceleration ramp type read from data flash memory.
  
  @param uint8_t type of ramp.
  @return void
 
*/
void UserConfigTask_UpdateThrottleAccelRampType(uint8_t rampType)
{
    userConfigData.Screen_ConfigData.Throttle_ConfigData.AccelRampType = rampType;
}

/**
  @brief Function to get throttle acceleration ramp argument 1 read from data flash memory.
  
  @param  void
  @return uint16_t argument 1
 
*/
uint16_t UserConfigTask_GetThrottleAccelRampArg1(void)
{
    return userConfigData.Screen_ConfigData.Throttle_ConfigData.AccelRampArg1;      
}

/**
  @brief Function to update the throttle acceleration ramp argument 1 read from data flash memory.
  
  @param uint16_t argument 1.
  @return void
 
*/
void UserConfigTask_UpdateThrottleAccelRampArg1(uint16_t arg1)
{
    userConfigData.Screen_ConfigData.Throttle_ConfigData.AccelRampArg1 = arg1; 
}

/**
  @brief Function to get Battery Full Voltage
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Full Voltage in Volts x100

*/
uint16_t UserConfigTask_GetBatteryFullVoltage(void)
{
    return userConfigData.Battery_ConfigData.FullVoltage;
}

/**
  @brief Function to update Battery Full Voltage
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Full Voltage in Volts x100
  @return void

*/
void UserConfigTask_UpdateBatteryFullVoltage(uint16_t value)
{   
    if(value > userConfigData.Battery_ConfigData.EmptyVoltage)
    {        
        userConfigData.Battery_ConfigData.FullVoltage = value;
    }
}

/**
  @brief Function to get Battery Empty Voltage 
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Empty Voltage in Volts x100

*/
uint16_t UserConfigTask_GetBatteryEmptyVoltage(void)
{
    return userConfigData.Battery_ConfigData.EmptyVoltage;
}

/**
  @brief Function to update Battery Empty Voltage
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Empty Voltage in Volts x100
  @return void

*/
void UserConfigTask_UpdateBatteryEmptyVoltage(uint16_t value)
{         
    if(value < userConfigData.Battery_ConfigData.FullVoltage)
    {
        userConfigData.Battery_ConfigData.EmptyVoltage = (uint16_t)value;
    }
}

/**
  @brief Function to get Battery Max Peak DC Current
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Max Peak DC Current in Amps x100

*/
uint16_t UserConfigTask_GetBatteryMaxPeakDCCurrent(void)
{
    return userConfigData.Battery_ConfigData.MaxPeakDCCurrent;
}

/**
  @brief Function to update Battery Max Peak DC Current 
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Max Peak DC Current
  @return void

*/
void UserConfigTask_UpdateBatteryMaxPeakDCCurrent(uint16_t value)
{       
    if(value >= userConfigData.Battery_ConfigData.ContinuousDCCurrent)
    {
        userConfigData.Battery_ConfigData.MaxPeakDCCurrent = (uint16_t)value;
    }
}

/**
  @brief Function to get Battery Continuous DC Current
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Continuous DC Current Amps x100

*/
uint16_t UserConfigTask_GetBatteryContinuousDCCurrent(void)
{
    return userConfigData.Battery_ConfigData.ContinuousDCCurrent;
}

/**
  @brief Function to update Battery Continuous DC Current
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Continuous DC Current Amps x100
  @return void

*/
void UserConfigTask_UpdateBatteryContinuousDCCurrent(uint16_t value)
{       
    if(value <= userConfigData.Battery_ConfigData.MaxPeakDCCurrent)
    {
        userConfigData.Battery_ConfigData.ContinuousDCCurrent = (uint16_t)value;
    }
}


/**
  @brief Function to get Battery Peak Current Derating Duration
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Peak Current Derating Duration Seconds x10

*/
uint16_t UserConfigTask_GetBatteryPeakCurrentDeratingDuration(void)
{
    return userConfigData.Battery_ConfigData.PeakCurrentDeratingDuration;
}

/**
  @brief Function to update Battery Peak Current Derating Duration
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery PeakCurrent Derating Duration Seconds x10
  @return void

*/
void UserConfigTask_UpdateBatteryPeakCurrentDeratingDuration(uint16_t value)
{       
    userConfigData.Battery_ConfigData.PeakCurrentDeratingDuration = (uint16_t)value;
}

/**
  @brief Function to get Battery Peak Current Max Duration
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Peak Current Max Duration Seconds x10

*/
uint16_t UserConfigTask_GetBatteryPeakCurrentMaxDuration(void)
{
    return userConfigData.Battery_ConfigData.PeakCurrentMaxDuration;
}

/**
  @brief Function to update Battery Peak Current Max Duration
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Peak Current Max Duration Seconds x10
  @return void

*/
void UserConfigTask_UpdateBatteryPeakCurrentMaxDuration(uint16_t value)
{       
    userConfigData.Battery_ConfigData.PeakCurrentMaxDuration = (uint16_t)value;
}

/**
  @brief Function to get FilterSpeed value
  read from data flash memory.
  @param uint8_t index number of the speed to get.
  @return uint8_t Speed value

*/
uint8_t UserConfigTask_GetFilterSpeed(uint8_t index)
{
    //
    if (index < FILTERSPEED_ARRAY_SIZE)
    {
        return userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index];
    }
    else
    {
        return userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[0];
    }   
}

/**
  @brief Function to update FilterSpeed value
  read from data flash memory.
  @param uint8_t index number of the speed to get.
  @param uint8_t speed value to be passed into FilterSpeed.
  @return void

*/
void UserConfigTask_UpdateFilterSpeed(uint8_t index, uint8_t value)
{
    //check speed value condtions.
    if ((index < FILTERSPEED_ARRAY_SIZE) && (value <= 75) && (value >= 1))
    {   
        //used to test speed condition, the index above must to bigger than the (index - 1).
        //ex: index 1 needs to have a speed value bigger than speed of index 0.
        switch(index)
        {
            case 0:
                //speed associated with index 0 must to be smaller than speed associated with the next index.
                if (userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index] < userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index + 1])
                {
                    userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index] = value;
                }     
            break;
            
            case 1:
                //speed associated with index 1 must to be bigger than speed associated with the previous index.
                if (userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index] > userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index - 1])
                {
                    userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.FilterSpeed[index] = value;
                }
            break;
            //nothing must to be done.    
            default:

            break;
        }
    }
}

/**
  @brief Function to get bw filter value
  read from data flash memory.
  @param uint8_t index number of the bw filter to get.
  @param uint8_t type of the filer to be get.
  @return uint8_t bw filter value.

*/
uint16_t UserConfigTask_GetFilterBwValue(uint8_t index, uint8_t bwType)
{
    //verify the condition to update the value.
    if ((index < BW_ARRAY_SIZE) && ((bwType == BW1) || (bwType == BW2)))
    {
        //state to decide what BW parameter will be returned.
        switch(bwType)
        {
            case BW1:

                return userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW1[index];  
            
            break;
            
            case BW2:
        
                return userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW2[index];

            break;
            
            default:
            
                return 0;
        
            break;   
        }        
    }
    else
    {
        return 0;
    }
}
    
/**
  @brief Function to chnage bw filter value
  read from data flash memory.
  @param uint8_t index of the bw filter to be changed.
  @param uint8_t type of the filer to be changed.
  @param uint16_t new bw filter value 
  @return void

*/
void UserConfigTask_UpdateFilterBwValue(uint8_t index, uint8_t bwType, uint16_t value)
{
    //verify the condition to update the value.
    if ((index < BW_ARRAY_SIZE) && (value > 0) && (value <= 0xFFFF) && ((bwType == BW1) || (bwType == BW2)))
    {
        //state to decide what BW parameter will be returned.
        switch(bwType)
        {
            case BW1:
        
                userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW1[index] = value;
            
            break;
            
            case BW2:
        
                userConfigData.PAS_ConfigData.PAS_Torque_Filter_Configuration.pasLowPassFilterBW2[index] = value;
            
            break;
            
            default:

        
            break;   
        }
    }    
}

/**
  @brief Function to get Minimum Torque Threshold
  read from data flash memory on running mode.
  
  @param void
  @return uint8_t a number that represent Minimum Torque Threshold.

*/
uint8_t UserConfigTask_GetPasTorqueRunningThreshold(void)
{
    return userConfigData.PAS_ConfigData.PAS_Running_Detection.pasTorqueRunningThreshold; 
}

/**
  @brief Function to update Minimum Torque Threshold
  read from data flash memory on running mode.
  
  @param uint8_t value to be passed into the  Minimum Torque Threshold.
  @return void
 
*/
void UserConfigTask_UpdatePasTorqueRunningThreshold(uint8_t value)
{
    //verify if value is in the range.
    if ((value <= 100) && (value >= 0))
    {
        userConfigData.PAS_ConfigData.PAS_Running_Detection.pasTorqueRunningThreshold = value;
    }        
}

/**
  @brief Function to get pasCadenceRunningNumbPulses
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent pasCadenceRunningNumbPulses.
*/
uint8_t UserConfigTask_GetPasCadenceRunningNumbPulses(void)
{
    return userConfigData.PAS_ConfigData.PAS_Running_Detection.pasCadenceRunningNumbPulses; 
}

/**
  @brief Function to update pasCadenceRunningNumbPulses
  read from data flash memory.
  
  @param uint8_t value to be passed into the pasCadenceRunningNumbPulses
  @return void
 
*/
void UserConfigTask_UpdatePasCadenceRunningNumbPulses(uint8_t value)
{
    //verify if value is in the range.
    if((value > 0) && (value <= 0xFF))
    {
        userConfigData.PAS_ConfigData.PAS_Running_Detection.pasCadenceRunningNumbPulses = value;
    }        
}

/**
  @brief Function to get time windows used to check the number of
  pulses detected from the cadence sensor read from data flash memory
  on run mode.
  
  @param void
  @return uint16_t a number that represent time windows used to check the number of
  pulses.
*/
uint16_t UserConfigTask_GetPasCadenceRunningWindows(void)
{
    return userConfigData.PAS_ConfigData.PAS_Running_Detection.pasCadenceRunningWindows;
}

/**
  @brief Function to update time windows used to check the number of
  pulses detected from the cadence sensor read from data flash memory
  on run mode.
  
  @param uint16_t value to be passed into the time windows used to check the number of
  pulses.
  @return void
 
*/
void UserConfigTask_UpdatePasCadenceRunningWindows(uint16_t value)
{
    //verify if value is in the range.
    //range on ms.
    if((value >= 200) && (value <= 0xFFFF))
    {
        userConfigData.PAS_ConfigData.PAS_Running_Detection.pasCadenceRunningWindows = value;
    }   
}

/**
  @brief Function to get pas algorithm detection used on running state
  read from data flash memory.
  
  @param void
  @return uint23_t pas algorithm detection used on running state.
*/
uint8_t UserConfigTask_GetPasAlgorithmRunning(void)
{
    return userConfigData.PAS_ConfigData.PAS_Running_Detection.PasAlgorithmRunning;
}

/**
  @brief Function to update pas algorithm detection used on running state
  read from data flash memory.
  
  @param uint32_t value equivalent to the pas algorithm detection that must be used.
  @return void
 
*/
void UserConfigTask_UpdatePasPasAlgorithmRunning(uint8_t value)
{
    //verify if value is in the range.
    //HybridOrSensorUse is the maximum vale, on this case 4.
    if((value >= 0) && (value <= MAX_PASALGORITHM_VALUE))
    {
        userConfigData.PAS_ConfigData.PAS_Running_Detection.PasAlgorithmRunning = value;
    }   
}

/**
  @brief Function to get pas acceleration ramp type read from data flash memory.
  
  @param  uint8_t pasLevel
  @return uint8_t type of ramp.
 
*/
uint8_t UserConfigTask_GetPasAccelRampType(uint8_t pasLevel)
{
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.PasAccelRampType[pasLevel-1];
    }
    else
    {
        return 0;
    }        
}

/**
  @brief Function to update the pas acceleration ramp type read from data flash memory.
  
  @param uint8_t pasLevel, uint8_t type of ramp.
  @return void
 
*/
void UserConfigTask_UpdatePasAccelRampType(uint8_t pasLevel, uint8_t rampType)
{
    if (0 < pasLevel && pasLevel < 10)
    {
         userConfigData.PAS_ConfigData.PasAccelRampType[pasLevel-1] = rampType;
    } 
}

/**
  @brief Function to get pas acceleration ramp argument 1 read from data flash memory.
  
  @param  uint8_t pasLevel
  @return uint16_t argument 1
 
*/
uint16_t UserConfigTask_GetPasAccelRampArg1(uint8_t pasLevel)
{
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.PasAccelRampArg1[pasLevel-1];
    }
    else
    {
        return 0;
    }        
}

/**
  @brief Function to update the pas acceleration ramp argument 1 read from data flash memory.
  
  @param uint8_t pasLevel, uint16_t argument 1.
  @return void
 
*/
void UserConfigTask_UpdatePasAccelRampArg1(uint8_t pasLevel, uint16_t arg1)
{
    if (0 < pasLevel && pasLevel < 10)
    {
         userConfigData.PAS_ConfigData.PasAccelRampArg1[pasLevel-1] = arg1;
    } 
}

/**
  @brief Function to get pas deceleration ramp type read from data flash memory.
  
  @param  uint8_t pasLevel
  @return uint8_t type of ramp.
 
*/
uint8_t UserConfigTask_GetPasDecelRampType(uint8_t pasLevel)
{
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.PasDecelRampType[pasLevel-1];
    }
    else
    {
        return 0;
    }        
}

/**
  @brief Function to update the pas deceleration ramp type read from data flash memory.
  
  @param uint8_t pasLevel, uint8_t type of ramp.
  @return void
 
*/
void UserConfigTask_UpdatePasDecelRampType(uint8_t pasLevel, uint8_t rampType)
{
    if (0 < pasLevel && pasLevel < 10)
    {
         userConfigData.PAS_ConfigData.PasDecelRampType[pasLevel-1] = rampType;
    } 
}

/**
  @brief Function to get pas deceleration ramp argument 1 read from data flash memory.
  
  @param  uint8_t pasLevel
  @return uint16_t argument 1
 
*/
uint16_t UserConfigTask_GetPasDecelRampArg1(uint8_t pasLevel)
{
    if (0 < pasLevel && pasLevel < 10)
    {
        return userConfigData.PAS_ConfigData.PasDecelRampArg1[pasLevel-1];
    }
    else
    {
        return 0;
    }        
}

/**
  @brief Function to update the pas deceleration ramp argument 1 read from data flash memory.
  
  @param uint8_t pasLevel, uint16_t argument 1.
  @return void
 
*/
void UserConfigTask_UpdatePasDecelRampArg1(uint8_t pasLevel, uint16_t arg1)
{
    if (0 < pasLevel && pasLevel < 10)
    {
         userConfigData.PAS_ConfigData.PasDecelRampArg1[pasLevel-1] = arg1;
    } 
}

/**
  @brief Function used to calculate a CRC 16 type using the same polynom 
  used by the bluetooth protocol.CCITT 16 bits polynom.
  
  @param uint8_t * data pointer to the data buffer where the CRC must be done.
  @param uint8_t length the length of the data to be calculated by the CRC algorithm.
  @return uint16_t return the calculated CRC.

*/
uint16_t UserConfigTask_CalculateCRC(uint8_t * buffer, uint8_t length)
{   
    uint8_t i;
    uint8_t n = 0;
    uint8_t value;
    uint16_t crc = 0x0000;
    //calculate the crc to all buffer length
    while(n < length)
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


/**
  @brief Function to get Throttle BlockOff Value
  read from data flash memory.
  
  @param void
  @return uint8_t Throttle Block Off

*/
uint8_t UserConfigTask_GetThrottleBlockOff(void)
{
    
    return userConfigData.Screen_ConfigData.Throttle_ConfigData.ThrottleBlock;
}

/**
  @brief Function to update Throttle BlockOff Value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Throttle Block Off
  @return void

*/
void UserConfigTask_UpdateThrottleBlockOff(uint8_t value)
{
    userConfigData.Screen_ConfigData.Throttle_ConfigData.ThrottleBlock = (uint8_t)value;
}

/**
  @brief Function to get Throttle Max Speed Value
  read from data flash memory.
  
  @param void
  @return uint8_t Throttle Max Speed

*/
uint8_t UserConfigTask_GetThrottleMaxSpeed(void)
{
    return userConfigData.Screen_ConfigData.Throttle_ConfigData.MaxSpeed;
}

/**
  @brief Function to update Throttle Max Speed Value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Throttle Max Speed
  @return void

*/
void UserConfigTask_UpdateThrottleMaxSpeed(uint8_t value)
{
    userConfigData.Screen_ConfigData.Throttle_ConfigData.MaxSpeed = (uint8_t)value;
}

/**
  @brief Function to get the mixed motor signal state(true or false)
  read from data flash memory.
  
  @param void
  @return bool true if motor signals are mixed, false if not.
*/
bool UserConfigTask_GetMotorMixedSignalState(void)
{
    return userConfigData.Screen_ConfigData.Motor_Signal_Parameters.motorMixedSignal; 
}

/**
  @brief Function to update the mixed motor signal state(true or false)
  read from data flash memory.
  
  @param bool new value
  @return none.
*/
void UserConfigTask_UpdateMotorMixedSignalState(bool value)
{
    userConfigData.Screen_ConfigData.Motor_Signal_Parameters.motorMixedSignal = value; 
}

/**
  @brief Function to get the minimal threshold to detect motor temperature
  and high logic 1 on wheel speed.
  
  @param void
  @return uint16_t minimum value used to detect max motor temperature
  and high level on wheel speed signal.
*/
uint16_t UserConfigTask_GetMinSignalThreshold(void)
{
    return userConfigData.Screen_ConfigData.Motor_Signal_Parameters.minSignalThreshold; 
}

/**
  @brief Function to update the minimal threshold to detect motor temperature
  and high logic 1 on wheel speed.
  
  @param uint16_t value is the minimal threshold to detect motor temperature
  and high logic 1 on wheel speed.
  @return none.
*/
void UserConfigTask_UpdateMinSignalThreshold(uint16_t value)
{
    userConfigData.Screen_ConfigData.Motor_Signal_Parameters.minSignalThreshold = value; 
}

/**
  @brief Function to get the MaxWheelSpeedPeriodUs, used to as limit to detect when
  wheel speed must be considered zero.
  
  @param void
  @return uint32_t maximum value used to as limit to detetc when
  wheel speed must be considered zero.
*/
uint32_t UserConfigTask_GetMaxWheelSpeedPeriodUs(void)
{
    return userConfigData.Screen_ConfigData.Motor_Signal_Parameters.maxWheelSpeedPeriodUs; 
}

/**
  @brief Function to update the MaxWheelSpeedPeriodUs, used to as limit to detect when
  wheel speed must be considered zero.
  
  @param uint32_t value maximum value used to as limit to detetc when
  wheel speed must be considered zero.
  @return none.
*/
void UserConfigTask_UpdateMaxWheelSpeedPeriodUs(uint32_t value)
{
    userConfigData.Screen_ConfigData.Motor_Signal_Parameters.maxWheelSpeedPeriodUs = value; 
}