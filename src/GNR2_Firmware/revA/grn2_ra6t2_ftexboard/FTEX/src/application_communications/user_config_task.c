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
#include "mc_config.h"

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
    .PAS_ConfigData.PASOverThrottle = PAS_OVER_THROTTLE,
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

    .PAS_ConfigData.PasSpeedThresholds[0] = (uint16_t)STARTUP_PAS_SPEED_THRESHOLD,
    .PAS_ConfigData.PasSpeedThresholds[1] = (uint16_t)RUNTIME_PAS_SPEED_THRESHOLD,
    .PAS_ConfigData.PasDetectionParameters[0] = TORQUE_STARTUP_VALUE_THRESHOLD,
    .PAS_ConfigData.PasDetectionParameters[1] = STARTUP_PULSE_NUMBER,
    .PAS_ConfigData.PasDetectionParameters[2] = STARTUP_TIME_WINDOW,
    .PAS_ConfigData.PasDetectionParameters[3] = RUNTIME_PULSE_NUMBER,
    .PAS_ConfigData.PasDetectionParameters[4] = RUNTIME_TIME_WINDOW,
    .PAS_ConfigData.PasCadenceAndOrTorque = CADENCE_AND_OR_TORQUE,
    .PAS_ConfigData.TorqueScalingPedalRPMActivated = (uint8_t)TORQUE_SCALING_PEDAL_RPM,
    .PAS_ConfigData.TorqueScalingPedalRPMParameters[0] = (uint16_t)MIN_RPM_SCALING,
    .PAS_ConfigData.TorqueScalingPedalRPMParameters[1] = (uint16_t)MAX_RPM_SCALING,
    .PAS_ConfigData.TorqueScalingPedalRPMParameters[2] = (uint16_t)GAIN_AT_MIN_RPM,
    .PAS_ConfigData.TorqueScalingPedalRPMParameters[3] = (uint16_t)GAIN_AT_MAX_RPM,
    .PAS_ConfigData.RampType = PAS_RAMP_SELECTION,
    .PAS_ConfigData.DynamicDecelerationRampParameters[0] = (uint16_t)DYNAMIC_DECEL_RAMP_START,
    .PAS_ConfigData.DynamicDecelerationRampParameters[1] = (uint16_t)DYNAMIC_DECEL_RAMP_END,
    .PAS_ConfigData.DynamicDecelerationRampParameters[2] = (uint16_t)DYNAMIC_DECEL_RAMP_POWER_MIN_SPEED,
    .PAS_ConfigData.DynamicDecelerationRampParameters[3] = (uint16_t)DYNAMIC_DECEL_RAMP_POWER_MAX_SPEED,
    .PAS_ConfigData.HighSpeedPowerLimitingRampParameters[0] = (uint16_t)HIGH_SPEED_POWER_LIMITING_RAMP_START,
    .PAS_ConfigData.HighSpeedPowerLimitingRampParameters[1] = (uint16_t)HIGH_SPEED_POWER_LIMITING_RAMP_END,
    .PAS_ConfigData.HighSpeedPowerLimitingRampParameters[2] = (uint16_t)HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MIN_SPEED,
    .PAS_ConfigData.HighSpeedPowerLimitingRampParameters[3] = (uint16_t)HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MAX_SPEED,
    
    .PAS_ConfigData.PasSensorConfig.pasNbMagnetsPerTurn = PAS_NB_MAGNETS_PER_TURN,
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
    .Battery_ConfigData.ContinuousDCCurrent = MAX_BMS_CONTINUOUS_CURRENT,
    .Battery_ConfigData.PeakCurrentDeratingDuration =  (MAX_TIME_BMS_TOLERANT - MAX_POWER_LIMIT_TIMEOUT),
    .Battery_ConfigData.PeakCurrentMaxDuration = MAX_POWER_LIMIT_TIMEOUT,
    .Screen_ConfigData.WalkmodeSpeed = PAS_LEVEL_SPEED_WALK,
    .Screen_ConfigData.WalkmodeMaxTorque = PAS_WALK_POWER_PERCENT,
    .Screen_ConfigData.MaxSpeed = VEHICLE_TOP_SPEED_KMH,
    #if WWS_USE_MOTOR_NBR_PER_ROTATION == true || EXTERNAL_WSS_NBR_PER_ROTATION == 0
        .Screen_ConfigData.WheelSpeedSensorNbrMagnets = MOTOR_WSS_NBR_PER_ROTATION,
    #else
        .Screen_ConfigData.WheelSpeedSensorNbrMagnets = EXTERNAL_WSS_NBR_PER_ROTATION,
    #endif
    .Screen_ConfigData.WheelDiameter = WHEEL_DIAMETER,
    .Screen_ConfigData.ScreenProtocol = SCREEN_PROTOCOL,
    .Screen_ConfigData.HeadLightDefault = POWERTRAIN_HEADLIGHT_DEFAULT,
    .Screen_ConfigData.TailLightDefault = POWERTRAIN_TAILLIGHT_DEFAULT,
    .Screen_ConfigData.TailLightBlinkOnBrake = REAR_LIGHT_BLINK_ON_BRAKE,
    .Screen_ConfigData.Throttle_ConfigData.ThrottleBlock = THROTTLE_BLOCK_OFF,
    .Screen_ConfigData.Throttle_ConfigData.MaxSpeed = THROTTLE_TOP_SPEED,
    .Screen_ConfigData.Motor_Signal_Parameters.motorMixedSignal = MOTOR_TEMP_MIXED,
    .Screen_ConfigData.Motor_Signal_Parameters.minSignalThreshold = MINIMUM_SIGNAL_THRESHOLD,
    .Screen_ConfigData.Motor_Signal_Parameters.maxWheelSpeedPeriodUs = MAX_WHEELSPEED_PERIOD_US,
    
    .Motor_Temperature_Parameters.motorSensorType = MOTOR_TEMP_SENSOR_TYPE,
    .Motor_Temperature_Parameters.motorNTCBetaCoef = MOTOR_NTC_BETA_COEFFICIENT,
    .Motor_Temperature_Parameters.motorNTCResistanceCoef = MOTOR_NTC_RESISTANCE_COEF_X_100,
    
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
    if(uCAL_Data_Flash_Open(userConfigHandle->pDataFlash_Handle) == true)
    {   
        //get date from data flash memory(backup)
        uCAL_Data_Flash_Read(userConfigHandle->pDataFlash_Handle, data, FLASH_HP_DF_BLOCK_4, USER_DATA_CONFIG_LENGTH);
        
        //calculate the crc to verify integrity of the data excluding crc bytes.
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
    uCAL_Data_Flash_Close(userConfigHandle->pDataFlash_Handle);
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
    if (uCAL_Data_Flash_Open(userConfigHandle->pDataFlash_Handle) == true)
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
    uCAL_Data_Flash_Close(userConfigHandle->pDataFlash_Handle);
    
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

    // Update the pedal torque sensor (PTS) offset values    
    PedalTorqueSensor_SetStartupOffsetMTSpeedKMH((uint16_t)UserConfigTask_GetPASTorqueDetectionValue());
       
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
    }
    
    paPowertrain->pPAS->sParameters.walkModeTorqueRatio = UserConfigTask_GetWalkmodeMaxTorque();
    paPowertrain->pPAS->sParameters.PASOverThrottle = UserConfigTask_GetPASOverThrottle();
    
    //update vehicle max speed(VEHICLE_TOP_SPEED_KMH).
    paPowertrain->sParameters.VehicleMaxSpeed = UserConfigTask_GetBikeMaxSpeed();

    //Get PAS sensor values
    PedalSpeedSensor_SetNumberOfMagnets(UserConfigTask_GetPasNbMagnetsPerTurn());  
    PedalTorqueSensor_SetSensorOffset(UserConfigTask_GetPasTorqueInputMin());
    PedalTorqueSensor_SetMaxTorqueValue(UserConfigTask_GetPasTorqueInputMax());

    PedalAssist_SetSpeedThresholds(paPowertrain->pPAS, UserConfigTask_GetPASStatupSpeedThreshold(), 
                                   UserConfigTask_GetPASRuntimeSpeedThreshold());
    PedalAssist_SetPASDetectionParameters(paPowertrain->pPAS, UserConfigTask_GetPASTorqueDetectionValue(), 
                                          UserConfigTask_GetPASStartupPulses(), UserConfigTask_GetPASStartupTimeWindow(),
                                          UserConfigTask_GetPASRuntimePulses(), UserConfigTask_GetPASRuntimeTimeWindow());
    PedalAssist_SetPASCadenceAndOrTorque(paPowertrain->pPAS, UserConfigTask_GetPASCadenceAndOrTorque());
    PedalAssist_SetTorqueScalingPedalRPM(paPowertrain->pPAS, UserConfigTask_GetPASTorqueScalingPedalRPMActivated());
    PedalAssist_SetTorqueScalingPedalRPMParameters(paPowertrain->pPAS, UserConfigTask_GetPASTorqueScalingMinRPM(), 
                                                   UserConfigTask_GetPASTorqueScalingMaxRPM(), UserConfigTask_GetPASTorqueScalingMinRPMGain(), 
                                                   UserConfigTask_GetPASTorqueScalingMaxRPMGain());
    PedalAssist_SetPASRampType(paPowertrain->pPAS, UserConfigTask_GetPASRampType());
    Ramps_SetDynamicDecelerationRampParameters(UserConfigTask_GetDynamicDecelerationRampStart(), UserConfigTask_GetDynamicDecelerationRampEnd(), 
                                               UserConfigTask_GetDynamicDecelerationMaxDeceleration(), UserConfigTask_GetDynamicDecelerationMinDeceleration());
    Ramps_SetHighSpeedPowerLimitingRampParameters(UserConfigTask_GetHighSpeedPowerLimitingRampStart(), UserConfigTask_GetHighSpeedPowerLimitingRampEnd(), 
                                                  UserConfigTask_GetHighSpeedPowerLimitingMinSpeedPower(), UserConfigTask_GetHighSpeedPowerLimitingMaxSpeedPower());

    //Throttle_ConfigData.walkMOdeSpeed(PAS_LEVEL_SPEED_WALK) is not passed
    //directly to any variable. Because of this is not updated here.

    MotorParameters.WheelSpeedSensorParameters.bWheelSpeedSensorNbrPerRotation = UserConfigTask_GetWheelSpeedSensorNbrMagnets();
    
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
     
    paPowertrain->pBatMonitorHandle->VBatMax = UserConfigTask_GetBatteryFullVoltage();
    paPowertrain->pBatMonitorHandle->VBatMin = UserConfigTask_GetBatteryEmptyVoltage();
    
    /******************************************************************************************/
    
    MDI_SetMaxBusCurrent(UserConfigTask_GetBatteryMaxPeakDCCurrent());

    MDI_SetMaxContinuousCurrent(UserConfigTask_GetBatteryContinuousDCCurrent());

    MDI_SetPowerFoldbackEndValue(UserConfigTask_GetBatteryPeakCurrentMaxDuration() + UserConfigTask_GetBatteryPeakCurrentDeratingDuration());
    
    MDI_SetPowerFoldbackRange(UserConfigTask_GetBatteryPeakCurrentDeratingDuration());
    
    MDI_SetMotorTempSensorType(UserConfigTask_GetMotorSensorType());
    MDI_SetMotorNTCBetaCoef(UserConfigTask_GetMotorNTCBetaCoef());
    MDI_SetMotorNTCResistanceCoef((float)UserConfigTask_GetMotorNTCResistanceCoef());
    
    /******************************************************************************************/
    
    for (uint8_t n = 0; n < FILTERSPEED_ARRAY_SIZE; n++)
    {
        PedalTorqueSensor_SetFilterSpeed(UserConfigTask_GetFilterSpeed(n), n);
    }
    
    for (uint8_t n = 0; n < BW_ARRAY_SIZE; n++)
    {
        PedalTorqueSensor_SetBWFilter1(UserConfigTask_GetFilterBwValue(n,BW1), n);
        PedalTorqueSensor_SetBWFilter2(UserConfigTask_GetFilterBwValue(n,BW2), n);
    }
    
    MDI_SetIsMotorSignalMixed(UserConfigTask_GetMotorMixedSignalState());
    MDI_SetMinSignalThresholdValueMixed(UserConfigTask_GetMinSignalThreshold());
    MDI_SetMaxWheelSpeedPeriodUsValueMixed(UserConfigTask_GetMaxWheelSpeedPeriodUs());
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
        if (value < DIGITAL33_0_25_VOLTS)
        {
            value = DIGITAL33_0_25_VOLTS;
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
        if (value < DIGITAL33_0_25_VOLTS)
        {
            value = DIGITAL33_0_25_VOLTS;
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
  @brief Function to get the number of magnets for the 
         wheel speed sensor
  @param void
  @return uint8_t value to be passed into the WheelSpeedSensorNbrMagnets

*/
uint8_t UserConfigTask_GetWheelSpeedSensorNbrMagnets(void)
{
    return userConfigData.Screen_ConfigData.WheelSpeedSensorNbrMagnets;
}

/**
  @brief Function to update the number of magnets for the 
         wheel speed sensor
  @param uint8_t value to be passed into the WheelSpeedSensorNbrMagnets
  @return void

*/
void UserConfigTask_UpdateWheelSpeedSensorNbrMagnets(uint8_t value)
{
    if ((value > 0) && (value <= 50))
    {
        userConfigData.Screen_ConfigData.WheelSpeedSensorNbrMagnets = value;
    }
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

/**
  @brief Function to get the MotorRpm, the current rpm of the motor.
  
  @param void
  @return int16_t value is th current motor rpm
*/
int16_t UserConfigTask_GetMotorRpm(VCI_Handle_t * pVController)
{
    return -MDI_GetAvrgMecSpeedUnit(pVController->pPowertrain->pMDI, M1);
}

/**
  @brief Function to get the MotorRpmWithGearRatio, the current rpm of the motor
         with the gear rato.
  @param void
  @return int16_t value is th current motor rpm with the gear ratio
*/
int16_t UserConfigTask_GetMotorRpmWithGearRatio(VCI_Handle_t * pVController)
{
    return (int16_t) (-MDI_GetAvrgMecSpeedUnit(pVController->pPowertrain->pMDI, M1) / MDI_GetMotorGearRatio(pVController->pPowertrain->pMDI));
}


/**
  @brief Function to get the PASOverThrottled, used to know what is 
         higher priority between PAS and throttle
  
  @param void
  @return uint8_t 1 to activate, 0 to desactivate
*/
uint8_t UserConfigTask_GetPASOverThrottle(void)
{
    return userConfigData.PAS_ConfigData.PASOverThrottle; 
}

/**
  @brief Function to update the PASOverThrottle, used to know what is 
         higher priority between PAS and throttle
  
  @param uint8_t 1 to activate, 0 to desactivate
  @return none.
*/
void UserConfigTask_UpdatePASOverThrottle(uint8_t value)
{
    if ( value > 1)
    {
        userConfigData.PAS_ConfigData.PASOverThrottle = 1;
    }
    else
    {
        userConfigData.PAS_ConfigData.PASOverThrottle = value;    
    }        
}

/**
  @brief Function to get the MotorSensorType, used to know if it is REAL_SENSOR or VIRTUAL_SENSOR
  
  @param void
  @return uint8_t 1 to VIRTUAL_SENSOR, 0 to REAL_SENSOR
*/
uint8_t UserConfigTask_GetMotorSensorType(void)
{
    return userConfigData.Motor_Temperature_Parameters.motorSensorType; 
}

/**
  @brief Function to update the MotorSensorType, used to update sensor type to REAL_SENSOR or VIRTUAL_SENSOR
  
  @param uint8_t 1 to VIRTUAL_SENSOR, 0 to REAL_SENSOR
  @return none.
*/
void UserConfigTask_UpdateMotorSensorType(uint8_t value)
{
    userConfigData.Motor_Temperature_Parameters.motorSensorType = value;    
}

/**
  @brief Function to get the Motor NTC Beta Coefficient, used to calculate motor temperature
  
  @param void
  @return uint16_t value of motor NTC Beta Coefficient
*/
uint16_t UserConfigTask_GetMotorNTCBetaCoef(void)
{
    return userConfigData.Motor_Temperature_Parameters.motorNTCBetaCoef; 
}

/**
  @brief Function to update the Motor NTC Beta Coefficient, used to calculate motor temperature
  
  @param uint16_t value of motor NTC Beta Coefficient
  @return none.
*/
void UserConfigTask_UpdateMotorNTCBetaCoef(uint16_t value)
{
    userConfigData.Motor_Temperature_Parameters.motorNTCBetaCoef = value;   
}

/**
  @brief Function to get the Motor NTC Rated Resistance, used to calculate motor temperature
  
  @param void
  @return uint16_t value of motor NTC rated resistance
*/
uint16_t UserConfigTask_GetMotorNTCResistanceCoef(void)
{
    return (uint16_t)(userConfigData.Motor_Temperature_Parameters.motorNTCResistanceCoef); 
}

/**
  @brief Function to update the Motor NTC Rated Resistance, used to calculate motor temperature
  
  @param uint16_t value of motor NTC rated resistance
  @return none.
*/
void UserConfigTask_UpdateMotorNTCResistanceCoef(uint16_t value)
{
    userConfigData.Motor_Temperature_Parameters.motorNTCResistanceCoef = value;  
}

/**
  @brief Get the PAS startup speed threshold
  @return uint16_t value of PAS statup speed threshold
*/
uint16_t UserConfigTask_GetPASStatupSpeedThreshold(void)
{
    return userConfigData.PAS_ConfigData.PasSpeedThresholds[0];
}

/**
  @brief Set the PAS startup speed threshold
  @param uint16_t Value of PAS startup speed threshold to set
  @return none.
*/
void UserConfigTask_SetPASStartupSpeedThreshold(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasSpeedThresholds[0] = value;
}

/**
  @brief Get the PAS runtime speed threshold
  @return uint16_t value of PAS runtime speed threshold
*/
uint16_t UserConfigTask_GetPASRuntimeSpeedThreshold(void)
{
    return userConfigData.PAS_ConfigData.PasSpeedThresholds[1];
}

/**
  @brief Set the PAS runtime speed threshold
  @param uint16_t Value of PAS runtime speed threshold to set
  @return none.
*/
void UserConfigTask_SetPASRuntimeSpeedThreshold(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasSpeedThresholds[1] = value;
}

/**
  @brief Get the PAS torque detection value
  @return uint16_t value of PAS runtime speed threshold
*/
uint16_t UserConfigTask_GetPASTorqueDetectionValue(void)
{
    return userConfigData.PAS_ConfigData.PasDetectionParameters[0];
}

/**
  @brief Set the PAS torque detection value
  @param uint16_t Value of PAS torque detection value to set
  @return none.
*/
void UserConfigTask_SetPASTorqueDetectionValue(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasDetectionParameters[0] = value;
}

/**
  @brief Get the PAS startup pulses number
  @return uint16_t value of PAS startup pulses number
*/
uint16_t UserConfigTask_GetPASStartupPulses(void)
{
    return userConfigData.PAS_ConfigData.PasDetectionParameters[1];
}

/**
  @brief Set the PAS startup pulses number
  @param uint16_t Value of PAS startup pulses number to set
  @return none.
*/
void UserConfigTask_SetPASStartupPulses(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasDetectionParameters[1] = value;
}

/**
  @brief Get the PAS startup time window
  @return uint16_t value of PAS startup timewindow
*/
uint16_t UserConfigTask_GetPASStartupTimeWindow(void)
{
    return userConfigData.PAS_ConfigData.PasDetectionParameters[2];
}

/**
  @brief Set the PAS startup time window
  @param uint16_t Value of PAS startup time window to set
  @return none.
*/
void UserConfigTask_SetPASStartupTimeWindow(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasDetectionParameters[2] = value;
}

/**
  @brief Get the PAS runtime pulses number
  @return uint16_t value of PAS runtime pulses number
*/
uint16_t UserConfigTask_GetPASRuntimePulses(void)
{
    return userConfigData.PAS_ConfigData.PasDetectionParameters[3];
}

/**
  @brief Set the PAS runtime pulses number
  @param uint16_t Value of PAS runtime pulses number to set
  @return none.
*/
void UserConfigTask_SetPASRuntimePulses(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasDetectionParameters[3] = value;
}

/**
  @brief Get the PAS runtime time window
  @return uint16_t value of PAS runtime timewindow
*/
uint16_t UserConfigTask_GetPASRuntimeTimeWindow(void)
{
    return userConfigData.PAS_ConfigData.PasDetectionParameters[4];
}

/**
  @brief Set the PAS runtime time window
  @param uint16_t Value of PAS runtime time window to set
  @return none.
*/
void UserConfigTask_SetPASRuntimeTimeWindow(uint16_t value)
{
    userConfigData.PAS_ConfigData.PasDetectionParameters[4] = value;
}

/**
  @brief Get the cadence AND/OR torque flag
  @return uint8_t value of cadence AND/OR torque flag
*/
uint8_t UserConfigTask_GetPASCadenceAndOrTorque(void)
{
    return userConfigData.PAS_ConfigData.PasCadenceAndOrTorque;
}

/**
  @brief Set the cadence AND/OR torque flag
  @param uint8_t Value of cadence AND/OR torque flag
  @return none.
*/
void UserConfigTask_SetPASCadenceAndOrTorque(uint8_t value)
{
    userConfigData.PAS_ConfigData.PasCadenceAndOrTorque = value;
}

/**
  @brief Get the torque scaling pedal RPM activated flag
  @return uint8_t value of torque scaling pedal RPM activated flag
*/
uint8_t UserConfigTask_GetPASTorqueScalingPedalRPMActivated(void)
{
    return userConfigData.PAS_ConfigData.TorqueScalingPedalRPMActivated;
}

/**
  @brief Set the torque scaling pedal RPM activated flag
  @param uint16_t Value of torque scaling pedal RPM activated flag
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingPedalRPMActivated(uint8_t value)
{
    userConfigData.PAS_ConfigData.TorqueScalingPedalRPMActivated = value;
}

/**
  @brief Get the PAS torque scaling min RPM
  @return uint16_t value of PAS torque scaling min RPM
*/
uint16_t UserConfigTask_GetPASTorqueScalingMinRPM(void)
{
    return userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[0];
}

/**
  @brief Set the PAS torque scaling min RPM
  @param uint16_t Value of PAS torque scaling min RPM
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMinRPM(uint16_t value)
{
    userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[0] = value;
}

/**
  @brief Get the PAS torque scaling max RPM
  @return uint16_t value of PAS torque scaling max RPM
*/
uint16_t UserConfigTask_GetPASTorqueScalingMaxRPM(void)
{
    return userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[1];
}

/**
  @brief Set the PAS torque scaling max RPM
  @param uint16_t Value of PAS torque scaling max RPM
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMaxRPM(uint16_t value)
{
    userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[1] = value;
}

/**
  @brief Get the PAS torque scaling min RPM gain
  @return uint16_t value of PAS torque scaling min RPM gain
*/
uint16_t UserConfigTask_GetPASTorqueScalingMinRPMGain(void)
{
    return userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[2];
}

/**
  @brief Set the PAS torque scaling min RPM gain
  @param uint16_t Value of PAS torque scaling min RPM gain
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMinRPMGain(uint16_t value)
{
    userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[2] = value;
}

/**
  @brief Get the PAS torque scaling max RPM gain
  @return uint16_t value of PAS torque scaling max RPM gain
*/
uint16_t UserConfigTask_GetPASTorqueScalingMaxRPMGain(void)
{
    return userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[3];
}

/**
  @brief Set the PAS torque scaling max RPM gain
  @param uint16_t Value of PAS torque scaling max RPM gain
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMaxRPMGain(uint16_t value)
{
    userConfigData.PAS_ConfigData.TorqueScalingPedalRPMParameters[3] = value;
}

/**
  @brief Get the PAS ramp type
  @return uint16_t value of PAS ramp type
*/
uint8_t UserConfigTask_GetPASRampType(void)
{
    return userConfigData.PAS_ConfigData.RampType;
}

/**
  @brief Set the PAS ramp type
  @param uint8_t Value of PAS ramp type
  @return none.
*/
void UserConfigTask_SetPASRampType(uint8_t value)
{
    userConfigData.PAS_ConfigData.RampType = value;
}

/**
  @brief Get the dynamic deceleration ramp start
  @return uint16_t value of dynamic deceleration ramp start
*/
uint16_t UserConfigTask_GetDynamicDecelerationRampStart(void)
{
    return userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[0];
}

/**
  @brief Set the dynamic deceleration ramp start
  @param uint16_t Value of dynamic deceleration ramp start
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationRampStart(uint16_t value)
{
    userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[0] = value;
}

/**
  @brief Get the dynamic deceleration ramp end
  @return uint16_t value of dynamic deceleration ramp end
*/
uint16_t UserConfigTask_GetDynamicDecelerationRampEnd(void)
{
    return userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[1];
}

/**
  @brief Set the dynamic deceleration ramp end
  @param uint16_t Value of dynamic deceleration ramp end
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationRampEnd(uint16_t value)
{
    userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[1] = value;
}

/**
  @brief Get the dynamic deceleration max deceleration
  @return uint16_t value of dynamic deceleration max deceleration
*/
uint16_t UserConfigTask_GetDynamicDecelerationMaxDeceleration(void)
{
    return userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[2];
}

/**
  @brief Set the dynamic deceleration ramp max deceleration
  @param uint16_t Value of dynamic deceleration ramp max deceleration
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationMaxDeceleration(uint16_t value)
{
    userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[2] = value;
}

/**
  @brief Get the dynamic deceleration min deceleration
  @return uint16_t value of dynamic deceleration min deceleration
*/
uint16_t UserConfigTask_GetDynamicDecelerationMinDeceleration(void)
{
    return userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[3];
}

/**
  @brief Set the dynamic deceleration ramp min deceleration
  @param uint16_t Value of dynamic deceleration ramp min deceleration
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationMinDeceleration(uint16_t value)
{
    userConfigData.PAS_ConfigData.DynamicDecelerationRampParameters[3] = value;
}

/**
  @brief Get the high speed power limiting ramp start
  @return uint16_t value of high speed power limiting ramp start
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingRampStart(void)
{
    return userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[0];
}

/**
  @brief Set the high speed power limiting ramp start
  @param uint16_t Value of high speed power limiting ramp start
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampStart(uint16_t value)
{
    userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[0] = value;
}

/**
  @brief Get the high speed power limiting ramp end
  @return uint16_t value of high speed power limiting ramp end
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingRampEnd(void)
{
    return userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[1];
}

/**
  @brief Set the high speed power limiting ramp end
  @param uint16_t Value of high speed power limiting ramp end
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampEnd(uint16_t value)
{
    userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[1] = value; 
}

/**
  @brief Get the high speed power limiting ramp min speed power
  @return uint16_t value of high speed power limiting ramp min speed power
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingMinSpeedPower(void)
{
    return userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[2];
}

/**
  @brief Set the high speed power limiting ramp min speed power
  @param uint16_t Value of high speed power limiting ramp min speed power
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampMinSpeedPower(uint16_t value)
{
    userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[2] = value;
}

/**
  @brief Get the high speed power limiting ramp max speed power
  @return uint16_t value of high speed power limiting ramp max speed power
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingMaxSpeedPower(void)
{
    return userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[3];
}

/**
  @brief Set the high speed power limiting ramp max speed power
  @param uint16_t Value of high speed power limiting ramp max speed power
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampMaxSpeedPower(uint16_t value)
{
    userConfigData.PAS_ConfigData.HighSpeedPowerLimitingRampParameters[3] = value;
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
