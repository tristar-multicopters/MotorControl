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

//
#define BW1             0
#define BW2             1

#define MAXTHROTTLEVOLTX100   330

#define VOLT_TO_DIGITAL    UINT16_MAX/MAXTHROTTLEVOLTX100

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
  @brief Function to get Pas Max Torque Ratio
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Pas Max Power
  on %(0 until 100).
*/
uint8_t UserConfigTask_GetPasMaxTorqueRatio(void);

/**
  @brief Function to update Pas Max Torque Ratio value
  read from data flash memory.
  
  @param uint8_t
  @return  void
*/
void UserConfigTask_UpdatePasMaxTorqueRatio(uint8_t value);

/**
  @brief Function to update Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Minimum Threshold Startup
  @return void
 
*/
uint8_t UserConfigTask_GetPasTorqueStartupSpeed(void);

/**
  @brief Function to get Torque Minimum Threshold Startup
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Minimum Threshold
  on %(0 until 100).
*/
void UserConfigTask_UpdatePasTorqueStartupSpeed(uint8_t value);

/**
  @brief Function to get Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Startup Offset Minimum Threshold Speed
  in RPM.
*/
uint8_t UserConfigTask_GetPasTorqueStartupThreshold(void);

/**
  @brief Function to update Startup Offset Minimum Threshold Speed
  read from data flash memory.
  
  @param uint8_t value to be passed into the Startup Offset Minimum Threshold Speed
  @return void
 
*/
void UserConfigTask_UpdatePasTorqueStartupThreshold(uint8_t value);


/**
  @brief Function to get pasCadenceStartupNumbPulses
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent pasCadenceStartupNumbPulses.
*/
uint16_t UserConfigTask_GetPasCadenceStartupNumbPulses(void);

/**
  @brief Function to update pasCadenceStartupNumbPulses
  read from data flash memory.
  
  @param uint16_t value to be passed into the pasCadenceStartupNumbPulses
  @return void
 
*/
void UserConfigTask_UpdatePasCadenceStartupNumbPulses(uint16_t value);

/**
  @brief Function to get time windows used to check the number of
  pulses detected from the cadence sensor read from data flash memory.
  
  @param void
  @return uint23_t a number that represent time windows used to check the number of
  pulses.
*/
uint32_t UserConfigTask_GetPasCadenceStartupWindows(void);

/**
  @brief Function to update time windows used to check the number of
  pulses detected from the cadence sensor read from data flash memory.
  
  @param uint32_t value to be passed into the time windows used to check the number of
  pulses.
  @return void
 
*/
void UserConfigTask_UpdatePasCadenceStartupWindows(uint32_t value);

/**
  @brief Function to get Torque Sensor Multiplier(GAIN)by PAS level
  read from data flash memory.
  
  @param void
  @return uint8_t a number that represent Torque Sensor Multiplier
  between 1 and 3.
*/
uint16_t UserConfigTask_GetTorqueSensorMultiplier(uint8_t pasLevel);

/**
  @brief Function to update Torque Sensor Multiplier value(GAIN) by PAS level
  read from data flash memory.
  
  @param uint8_t value to be passed into the Torque Sensor Multiplier
  @return void

*/
void UserConfigTask_UpdateTorqueSensorMultiplier(uint8_t pasLevel, uint16_t value);

/**
  @brief Function to get speed to the current Pas Level
  read from data flash memory.
  
  @param void
  @return uint8_t range will be defined.

*/
uint8_t UserConfigTask_GetPasLevelMinTorque(uint8_t pasLevel);

/**
  @brief Function to update torque Max Speed avlue
  read from data flash memory.
  
  @param uint8_t value to be passed into the
  @return void

*/
void UserConfigTask_UpdatePasLevelMinTorque(uint8_t value, uint8_t pasLevel);

/**
  @brief Function to get cadence Level Speed
  read from data flash memory.
  
  @param void
  @return uint8_t speed up to which this PAS level will give motor assistance
          range bewteen 0-40.

*/
uint8_t UserConfigTask_GetPasLevelSpeed(uint8_t pasLevel);

/**
  @brief Function to update speed to the current Pas Level
  read from data flash memory.
  
  @param uint8_t value to be passed into the
  @return void

*/
void UserConfigTask_UpdatePasLevelSpeed(uint8_t pasLevel, uint8_t value);

/**
  @brief Function to get pasLevelMaxTorque
  read from data flash memory.
  
  @param void
  @return uint8_t Percentage of max motor assistance that this PAS level will give
          range bewteen 0-x.

*/
uint8_t UserConfigTask_GetPasLevelMaxTorque(uint8_t pasLevel);

/**
  @brief Function to update pasLevelMaxTorque value
  read from data flash memory.
  
  @param uint8_t value to be passed into the pasLevelMaxTorque
  @return void

*/
void UserConfigTask_UpdatePasLevelMaxTorque(uint8_t pasLevel, uint8_t value);

/**
  @brief Function to get Pas Nb Magnets Per Turn
  read from data flash memory.
  
  @param void
  @return uint8_t Number of magnets per pedal rotation, cannot be 0

*/
uint8_t UserConfigTask_GetPasNbMagnetsPerTurn(void);

/**
  @brief Function to update Pas Nb Magnets Per Turn value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Pas Nb Pulse Per Turn cannot be 0
  @return void

*/
void UserConfigTask_UpdatePasNbMagnetsPerTurn(uint8_t value);

/**
  @brief Function to get Pas Torque Input Max
  read from data flash memory.
  
  @param void
  @return uint16_t Pas Torque Input Max

*/
uint16_t UserConfigTask_GetPasTorqueInputMax(void);

/**
  @brief Function to update Pas Torque Input Max value
  read from data flash memory.
  
  @param uint16_t value to be passed into the Pas Torque Input Max value
  @return void

*/
void UserConfigTask_UpdatePasTorqueInputMax(uint16_t value);

/**
  @brief Function to get Pas Torque Input Min
  read from data flash memory.
  
  @param void
  @return uint16_t Pas Torque Input Min

*/
uint16_t UserConfigTask_GetPasTorqueInputMin(void);

/**
  @brief Function to update Pas Torque Input Min
  read from data flash memory.
  
  @param uint16_t value to be passed into the Pas Torque Input Min
  @return void

*/
void UserConfigTask_UpdatePasTorqueInputMin(uint16_t value);

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
  @brief Function to get HeadLight Default
  read from data flash memory.
  
  @param void
  @return uint8_t HeadLight Default

*/
uint8_t UserConfigTask_GetHeadLightDefault(void);

/**
  @brief Function to update bike HeadLight Default value
  read from data flash memory.
  
  @param uint8_t value to be passed into the HeadLight Default
  @return void

*/
void UserConfigTask_UpdateHeadLightDefault(uint8_t value);

/**
  @brief Function to get TailLight Default
  read from data flash memory.
  
  @param void
  @return uint8_t TailLight Default

*/
uint8_t UserConfigTask_GetTailLightDefault(void);

/**
  @brief Function to update bike TailLight Default value
  read from data flash memory.
  
  @param uint8_t value to be passed into the TailLight Default
  @return void

*/
void UserConfigTask_UpdateTailLightDefault(uint8_t value);

/**
  @brief Function to get TailLight Blink On Brake
  read from data flash memory.
  
  @param void
  @return uint8_t TailLight Blink On Brake

*/
uint8_t UserConfigTask_GetTailLightBlinkOnBrake(void);

/**
  @brief Function to update bike TailLight Blink On Brake value
  read from data flash memory.
  
  @param uint8_t value to be passed into the TailLight Blink On Brake
  @return void

*/
void UserConfigTask_UpdateTailLightBlinkOnBrake(uint8_t value);

/**
  @brief Function to get Throttle Adc Offset
  read from data flash memory.
  
  @param void
  @return uint16_t Throttle Adc Offset

*/
uint16_t UserConfigTask_GetThrottleAdcOffset(void);

/**
  @brief Function to update Throttle Adc Offset
  read from data flash memory.
  
  @param uint16_t value to be passed into the Throttle Adc Offset
  @return void

*/
void UserConfigTask_UpdateThrottleAdcOffset(uint16_t value);

/**
  @brief Function to get Throttle Adc Max
  read from data flash memory.
  
  @param void
  @return uint16_t Throttle Adc Max

*/
uint16_t UserConfigTask_GetThrottleAdcMax(void);

/**
  @brief Function to update Throttle Adc Max
  read from data flash memory.
  
  @param uint16_t value to be passed into the Throttle Adc Max
  @return void

*/
void UserConfigTask_UpdateThrottleAdcMax(uint16_t value);

/**
  @brief Function to get Battery Full Voltage
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Full Voltage in Volts x100

*/
uint16_t UserConfigTask_GetBatteryFullVoltage(void);

/**
  @brief Function to update Battery Full Voltage
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Full Voltage in Volts x100
  @return void

*/
void UserConfigTask_UpdateBatteryFullVoltage(uint16_t value);

/**
  @brief Function to get Battery Empty Voltage 
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Empty Voltage in Volts x100

*/
uint16_t UserConfigTask_GetBatteryEmptyVoltage(void);

/**
  @brief Function to update Battery Empty Voltage
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Empty Voltage in Volts x100
  @return void

*/
void UserConfigTask_UpdateBatteryEmptyVoltage(uint16_t value);

/**
  @brief Function to get Battery Max Peak DC Current
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Max Peak DC Current in Amps x100

*/
uint16_t UserConfigTask_GetBatteryMaxPeakDCCurrent(void);

/**
  @brief Function to update Battery Max Peak DC Current 
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Max Peak DC Current in Amps x100
  @return void

*/
void UserConfigTask_UpdateBatteryMaxPeakDCCurrent(uint16_t value);

/**
  @brief Function to get Battery Continuous DC Current
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Continuous DC Current Amps x100

*/
uint16_t UserConfigTask_GetBatteryContinuousDCCurrent(void);

/**
  @brief Function to update Battery Continuous DC Current
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Continuous DC Current Amps x100
  @return void

*/
void UserConfigTask_UpdateBatteryContinuousDCCurrent(uint16_t value);

/**
  @brief Function to get Battery Peak Current Derating Duration
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Peak Current Derating Duration In Seconds x10

*/
uint16_t UserConfigTask_GetBatteryPeakCurrentDeratingDuration(void);

/**
  @brief Function to update Battery Peak Current Derating Duration
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery PeakCurrent Derating Duration Seconds x10
  @return void

*/
void UserConfigTask_UpdateBatteryPeakCurrentDeratingDuration(uint16_t value);

/**
  @brief Function to get Battery Peak Current Max Duration
  read from data flash memory.
  
  @param void
  @return uint16_t Battery Peak Current Max Duration Seconds x10

*/
uint16_t UserConfigTask_GetBatteryPeakCurrentMaxDuration(void);

/**
  @brief Function to update Battery Peak Current Max Duration
  read from data flash memory.
  
  @param uint16_t value to be passed into the Battery Peak Current Max Duration Seconds x10
  @return void

*/
void UserConfigTask_UpdateBatteryPeakCurrentMaxDuration(uint16_t value);

/**
  @brief Function to get FilterSpeed value
  read from data flash memory.
  @param uint8_t index number of the speed to get.
  @return uint8_t Speed value

*/
uint8_t UserConfigTask_GetFilterSpeed(uint8_t index);

/**
  @brief Function to update FilterSpeed value
  read from data flash memory.
  @param uint8_t index number of the speed to get.
  @param uint8_t speed value to be passed into FilterSpeed.
  @return void

*/
void UserConfigTask_UpdateFilterSpeed(uint8_t index, uint8_t value);

/**
  @brief Function to get bw filter value
  read from data flash memory.
  @param uint8_t index number of the bw filter to get.
  @param uint8_t type of the filer to be get.
  @return uint8_t bw filter value.

*/
uint16_t UserConfigTask_GetFilterBwValue(uint8_t index, uint8_t bwType);

/**
  @brief Function to chnage bw filter value
  read from data flash memory.
  @param uint8_t index of the bw filter to be changed.
  @param uint8_t type of the filer to be changed.
  @param uint16_t new bw filter value 
  @return void

*/
void UserConfigTask_UpdateFilterBwValue(uint8_t index, uint8_t bwType, uint16_t value);

/**
  @brief Function used to calculate a CRC 16 type using the same polynom 
  used by the bluetooth protocol.CCITT 16 bits polynom.
  
  @param uint8_t * data pointer to the data buffer where the CRC must be done.
  @param uint8_t length the length of the data to be calculated by the CRC algorithm.
  @return uint16_t return the calculated CRC.

*/
uint16_t UserConfigTask_CalculateCRC(uint8_t * buffer, uint8_t length);

#endif