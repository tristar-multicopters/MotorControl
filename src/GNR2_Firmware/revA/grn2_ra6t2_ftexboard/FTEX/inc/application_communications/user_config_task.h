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



#include "comm_config.h"
#include "vc_parameters.h"


/*********************************************
                Defines
*********************************************/
#define PAS_0  0
#define PAS_1  1
#define PAS_2  2
#define PAS_3  3
#define PAS_4  4
#define PAS_5  5
#define PAS_6  6
#define PAS_7  7
#define PAS_8  8
#define PAS_9  9

#define NUMBER_OF_BYTES_IN_THE_BLOCK    64U

//first byte used to show that data memory
//has user configuration
#define ID0_DATA_FLASH                 0x01

//second byte used to show that data memory
//has user configuration
#define ID1_DATA_FLASH                 0x03

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
void UserConfigTask_UpdatePasLevelMinTorque(uint8_t pasLevel, uint8_t value);

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
uint8_t UserConfigTask_GetWalkmodeSpeed(void);

/**
  @brief Function to update bike speed on walk mode value
  read from data flash memory.
  
  @param uint8_t value to be passed into the walk mode speed
  @return void

*/
void UserConfigTask_UpdateWalkmodeSpeed(uint8_t value);

/**
  @brief Function to get max torque on walk mode
  read from data flash memory.
  
  @param void
  @return uint8_t max torque in % that the walk mode of the vehicle goes up to.

*/
uint8_t UserConfigTask_GetWalkmodeMaxTorque(void);

/**
  @brief Function to update max torque on walk mode value
  read from data flash memory.
  
  @param uint8_t value to be passed into the walk mode max torque in %
  @return void

*/
void UserConfigTask_UpdateWalkmodeMaxTorque(uint8_t value);

/**
  @brief Function to get walkmode acceleration ramp type read from data flash memory.
  
  @param  void
  @return uint8_t type of ramp.
 
*/
uint8_t UserConfigTask_GetWalkmodeAccelRampType(void);

/**
  @brief Function to update the walkmode acceleration ramp type read from data flash memory.
  
  @param uint8_t type of ramp.
  @return void
 
*/
void UserConfigTask_UpdateWalkmodeAccelRampType(uint8_t rampType);

/**
  @brief Function to get walkmode acceleration ramp argument 1 read from data flash memory.
  
  @param  void
  @return uint16_t argument 1
 
*/
uint16_t UserConfigTask_GetWalkmodeAccelRampArg1(void);

/**
  @brief Function to update the walkmode acceleration ramp argument 1 read from data flash memory.
  
  @param uint16_t argument 1.
  @return void
 
*/
void UserConfigTask_UpdateWalkmodeAccelRampArg1(uint16_t arg1);

/**
  @brief Function to get the number of magnets for the 
         wheel speed sensor
  @param void
  @return uint8_t value to be passed into the WheelSpeedSensorNbrMagnets

*/
uint8_t UserConfigTask_GetWheelSpeedSensorNbrMagnets(void);

/**
  @brief Function to update the number of magnets for the 
         wheel speed sensor
  @param uint8_t value to be passed into the WheelSpeedSensorNbrMagnets
  @return void

*/
void UserConfigTask_UpdateWheelSpeedSensorNbrMagnets(uint8_t value);

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
  @brief Function to get throttle acceleration ramp type read from data flash memory.
  
  @param  void
  @return uint8_t type of ramp.
 
*/
uint8_t UserConfigTask_GetThrottleAccelRampType(void);

/**
  @brief Function to update the throttle acceleration ramp type read from data flash memory.
  
  @param uint8_t type of ramp.
  @return void
 
*/
void UserConfigTask_UpdateThrottleAccelRampType(uint8_t rampType);

/**
  @brief Function to get throttle acceleration ramp argument 1 read from data flash memory.
  
  @param  void
  @return uint16_t argument 1
 
*/
uint16_t UserConfigTask_GetThrottleAccelRampArg1(void);

/**
  @brief Function to update the throttle acceleration ramp argument 1 read from data flash memory.
  
  @param uint16_t argument 1.
  @return void
 
*/
void UserConfigTask_UpdateThrottleAccelRampArg1(uint16_t arg1);

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
  @brief Function to get Throttle BlockOff Value
  read from data flash memory.
  
  @param void
  @return uint8_t Throttle Block Off

*/
uint8_t UserConfigTask_GetThrottleBlockOff(void);

/**
  @brief Function to update Throttle BlockOff Value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Throttle Block Off
  @return void

*/
void UserConfigTask_UpdateThrottleBlockOff(uint8_t value);

/**
  @brief Function to get Throttle Max Speed Value
  read from data flash memory.
  
  @param void
  @return uint8_t Throttle Max Speed

*/
uint8_t UserConfigTask_GetThrottleMaxSpeed(void);

/**
  @brief Function to update Throttle Max Speed Value
  read from data flash memory.
  
  @param uint8_t value to be passed into the Throttle Max Speed
  @return void

*/
void UserConfigTask_UpdateThrottleMaxSpeed(uint8_t value);

/**
  @brief Function to get the mixed motor signal state(true or false)
  read from data flash memory.
  
  @param void
  @return bool true if motor signals are mixed, false if not.
*/
bool UserConfigTask_GetMotorMixedSignalState(void);

/**
  @brief Function to update the mixed motor signal state(true or false)
  read from data flash memory.
  
  @param bool new value
  @return none.
*/
void UserConfigTask_UpdateMotorMixedSignalState(bool value);

/**
  @brief Function to get the minimal threshold to detect motor temperature
  and high logic 1 on wheel speed.
  
  @param void
  @return uint16_t minimum value used to detect max motor temperature
  and high level on wheel speed signal.
*/
uint16_t UserConfigTask_GetMinSignalThreshold(void);

/**
  @brief Function to update the minimal threshold to detect motor temperature
  and high logic 1 on wheel speed..
  
  @param uint16_t value is the minimal threshold to detect motor temperature
  and high logic 1 on wheel speed.
  @return none.
*/
void UserConfigTask_UpdateMinSignalThreshold(uint16_t value);

/**
  @brief Function to get the MaxWheelSpeedPeriodUs, used to as limit to detect when
  wheel speed must be considered zero.
  
  @param void
  @return uint32_t maximum value used to as limit to detect when
  wheel speed must be considered zero.
*/
uint32_t UserConfigTask_GetMaxWheelSpeedPeriodUs(void);

/**
  @brief Function to update the MaxWheelSpeedPeriodUs, used to as limit to detect when
  wheel speed must be considered zero.
  
  @param uint32_t value maximum value used to as limit to detect when
  wheel speed must be considered zero.
  @return none.
*/
void UserConfigTask_UpdateMaxWheelSpeedPeriodUs(uint32_t value);

/*
  @brief Function to get the MotorRpm, the current rpm of the motor.
  
  @param void
  @return int16_t value is th current motor rpm
*/
int16_t UserConfigTask_GetMotorRpm(VCI_Handle_t * pVController);

/**
  @brief Function to get the MotorRpmWithGearRatio, the current rpm of the motor
         with the gear rato.
  @param void
  @return int16_t value is th current motor rpm with the gear ratio
*/
int16_t UserConfigTask_GetMotorRpmWithGearRatio(VCI_Handle_t * pVController);

/**
  @brief Function to get the PASOverThrottle, used to know what is 
         higher priority between PAS and throttle
  
  @param void
  @return uint8_t 1 to activate, 0 to desactivate
*/
uint8_t UserConfigTask_GetPASOverThrottle(void);

/**
  @brief Function to update the PASOverThrottle, used to know what is 
         higher priority between PAS and throttle
  
  @param uint8_t 1 to activate, 0 to desactivate
  @return none.
*/
void UserConfigTask_UpdatePASOverThrottle(uint8_t value);

/**
  @brief Function to get the MotorSensorType, used to know if it is REAL_SENSOR or VIRTUAL_SENSOR
  
  @param void
  @return uint8_t 1 to VIRTUAL_SENSOR, 0 to REAL_SENSOR
*/
uint8_t UserConfigTask_GetMotorSensorType(void);

/**
  @brief Function to update the MotorSensorType, used to update sensor type to REAL_SENSOR or VIRTUAL_SENSOR
  
  @param uint8_t 1 to VIRTUAL_SENSOR, 0 to REAL_SENSOR
  @return none.
*/
void UserConfigTask_UpdateMotorSensorType(uint8_t value);

/**
  @brief Function to get the Motor NTC Beta Coefficient, used to calculate motor temperature
  
  @param void
  @return uint16_t value of motor NTC Beta Coefficient
*/
uint16_t UserConfigTask_GetMotorNTCBetaCoef(void);

/**
  @brief Function to update the Motor NTC Beta Coefficient, used to calculate motor temperature
  
  @param uint16_t value of motor NTC Beta Coefficient
  @return none.
*/
void UserConfigTask_UpdateMotorNTCBetaCoef(uint16_t value);

/**
  @brief Function to get the Motor NTC Rated Resistance, used to calculate motor temperature
  
  @param void
  @return uint16_t value of motor NTC rated resistance
*/
uint16_t UserConfigTask_GetMotorNTCResistanceCoef(void);

/**
  @brief Function to update the Motor NTC Rated Resistance, used to calculate motor temperature
  
  @param uint16_t value of motor NTC rated resistance
  @return none.
*/
void UserConfigTask_UpdateMotorNTCResistanceCoef(uint16_t value);

/**
  @brief Get the PAS startup speed threshold
  @return uint16_t value of PAS statup speed threshold
*/
uint16_t UserConfigTask_GetPASStatupSpeedThreshold(void);

/**
  @brief Set the PAS startup speed threshold
  @param uint16_t Value of PAS startup speed threshold to set
  @return none.
*/
void UserConfigTask_SetPASStartupSpeedThreshold(uint16_t value);

/**
  @brief Get the PAS runtime speed threshold
  @return uint16_t value of PAS runtime speed threshold
*/
uint16_t UserConfigTask_GetPASRuntimeSpeedThreshold(void);

/**
  @brief Set the PAS runtime speed threshold
  @param uint16_t Value of PAS runtime speed threshold to set
  @return none.
*/
void UserConfigTask_SetPASRuntimeSpeedThreshold(uint16_t value);

/**
  @brief Get the PAS torque detection value
  @return uint16_t value of PAS runtime speed threshold
*/
uint16_t UserConfigTask_GetPASTorqueDetectionValue(void);

/**
  @brief Set the PAS torque detection value
  @param uint16_t Value of PAS torque detection value to set
  @return none.
*/
void UserConfigTask_SetPASTorqueDetectionValue(uint16_t value);

/**
  @brief Get the PAS startup pulses number
  @return uint16_t value of PAS startup pulses number
*/
uint16_t UserConfigTask_GetPASStartupPulses(void);

/**
  @brief Set the PAS startup pulses number
  @param uint16_t Value of PAS startup pulses number to set
  @return none.
*/
void UserConfigTask_SetPASStartupPulses(uint16_t value);

/**
  @brief Get the PAS startup time window
  @return uint16_t value of PAS startup timewindow
*/
uint16_t UserConfigTask_GetPASStartupTimeWindow(void);

/**
  @brief Set the PAS startup time window
  @param uint16_t Value of PAS startup time window to set
  @return none.
*/
void UserConfigTask_SetPASStartupTimeWindow(uint16_t value);

/**
  @brief Get the PAS runtime pulses number
  @return uint16_t value of PAS runtime pulses number
*/
uint16_t UserConfigTask_GetPASRuntimePulses(void);

/**
  @brief Set the PAS runtime pulses number
  @param uint16_t Value of PAS runtime pulses number to set
  @return none.
*/
void UserConfigTask_SetPASRuntimePulses(uint16_t value);

/**
  @brief Get the PAS runtime time window
  @return uint16_t value of PAS runtime timewindow
*/
uint16_t UserConfigTask_GetPASRuntimeTimeWindow(void);

/**
  @brief Set the PAS runtime time window
  @param uint8_t Value of PAS runtime time window to set
  @return none.
*/
void UserConfigTask_SetPASRuntimeTimeWindow(uint16_t value);

/**
  @brief Get the cadence AND/OR torque flag
  @return uint8_t value of cadence AND/OR torque flag
*/
uint8_t UserConfigTask_GetPASCadenceAndOrTorque(void);

/**
  @brief Set the cadence AND/OR torque flag
  @param uint16_t Value of cadence AND/OR torque flag
  @return none.
*/
void UserConfigTask_SetPASCadenceAndOrTorque(uint8_t value);

/**
  @brief Get the torque scaling pedal RPM activated flag
  @return uint8_t value of torque scaling pedal RPM activated flag
*/
uint8_t UserConfigTask_GetPASTorqueScalingPedalRPMActivated(void);

/**
  @brief Set the torque scaling pedal RPM activated flag
  @param uint16_t Value of torque scaling pedal RPM activated flag
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingPedalRPMActivated(uint8_t value);

/**
  @brief Get the PAS torque scaling min RPM
  @return uint16_t value of PAS torque scaling min RPM
*/
uint16_t UserConfigTask_GetPASTorqueScalingMinRPM(void);

/**
  @brief Set the PAS torque scaling min RPM
  @param uint16_t Value of PAS torque scaling min RPM
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMinRPM(uint16_t value);

/**
  @brief Get the PAS torque scaling max RPM
  @return uint16_t value of PAS torque scaling max RPM
*/
uint16_t UserConfigTask_GetPASTorqueScalingMaxRPM(void);

/**
  @brief Set the PAS torque scaling max RPM
  @param uint16_t Value of PAS torque scaling max RPM
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMaxRPM(uint16_t value);

/**
  @brief Get the PAS torque scaling min RPM gain
  @return uint16_t value of PAS torque scaling min RPM gain
*/
uint16_t UserConfigTask_GetPASTorqueScalingMinRPMGain(void);

/**
  @brief Set the PAS torque scaling min RPM gain
  @param uint16_t Value of PAS torque scaling min RPM gain
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMinRPMGain(uint16_t value);

/**
  @brief Get the PAS torque scaling max RPM gain
  @return uint16_t value of PAS torque scaling max RPM gain
*/
uint16_t UserConfigTask_GetPASTorqueScalingMaxRPMGain(void);

/**
  @brief Set the PAS torque scaling max RPM gain
  @param uint16_t Value of PAS torque scaling max RPM gain
  @return none.
*/
void UserConfigTask_SetPASTorqueScalingMaxRPMGain(uint16_t value);

/**
  @brief Get the PAS ramp type
  @return uint16_t value of PAS ramp type
*/
uint8_t UserConfigTask_GetPASRampType(void);

/**
  @brief Set the PAS ramp type
  @param uint8_t Value of PAS ramp type
  @return none.
*/
void UserConfigTask_SetPASRampType(uint8_t value);

/**
  @brief Get the dynamic deceleration ramp start
  @return uint16_t value of dynamic deceleration ramp start
*/
uint16_t UserConfigTask_GetDynamicDecelerationRampStart(void);

/**
  @brief Set the dynamic deceleration ramp start
  @param uint16_t Value of dynamic deceleration ramp start
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationRampStart(uint16_t value);

/**
  @brief Get the dynamic deceleration ramp end
  @return uint16_t value of dynamic deceleration ramp end
*/
uint16_t UserConfigTask_GetDynamicDecelerationRampEnd(void);

/**
  @brief Set the dynamic deceleration ramp end
  @param uint16_t Value of dynamic deceleration ramp end
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationRampEnd(uint16_t value);

/**
  @brief Get the dynamic deceleration max deceleration
  @return uint16_t value of dynamic deceleration max deceleration
*/
uint16_t UserConfigTask_GetDynamicDecelerationMaxDeceleration(void);

/**
  @brief Set the dynamic deceleration ramp max deceleration
  @param uint16_t Value of dynamic deceleration ramp max deceleration
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationMaxDeceleration(uint16_t value);

/**
  @brief Get the dynamic deceleration min deceleration
  @return uint16_t value of dynamic deceleration min deceleration
*/
uint16_t UserConfigTask_GetDynamicDecelerationMinDeceleration(void);

/**
  @brief Set the dynamic deceleration ramp min deceleration
  @param uint16_t Value of dynamic deceleration ramp min deceleration
  @return none.
*/
void UserConfigTask_SetDynamicDecelerationMinDeceleration(uint16_t value);

/**
  @brief Get the high speed power limiting ramp start
  @return uint16_t value of high speed power limiting ramp start
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingRampStart(void);

/**
  @brief Set the high speed power limiting ramp start
  @param uint16_t Value of high speed power limiting ramp start
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampStart(uint16_t value);

/**
  @brief Get the high speed power limiting ramp end
  @return uint16_t value of high speed power limiting ramp end
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingRampEnd(void);

/**
  @brief Set the high speed power limiting ramp end
  @param uint16_t Value of high speed power limiting ramp end
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampEnd(uint16_t value);

/**
  @brief Get the high speed power limiting ramp min speed power
  @return uint16_t value of high speed power limiting ramp min speed power
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingMinSpeedPower(void);

/**
  @brief Set the high speed power limiting ramp min speed power
  @param uint16_t Value of high speed power limiting ramp min speed power
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampMinSpeedPower(uint16_t value);

/**
  @brief Get the high speed power limiting ramp max speed power
  @return uint16_t value of high speed power limiting ramp max speed power
*/
uint16_t UserConfigTask_GetHighSpeedPowerLimitingMaxSpeedPower(void);

/**
  @brief Set the high speed power limiting ramp max speed power
  @param uint16_t Value of high speed power limiting ramp max speed power
  @return none.
*/
void UserConfigTask_SetHighSpeedPowerLimitingRampMaxSpeedPower(uint16_t value);

/**
  @brief Function used to calculate a CRC 16 type using the same polynom 
  used by the bluetooth protocol.CCITT 16 bits polynom.
  
  @param uint8_t * data pointer to the data buffer where the CRC must be done.
  @param uint8_t length the length of the data to be calculated by the CRC algorithm.
  @return uint16_t return the calculated CRC.

*/
uint16_t UserConfigTask_CalculateCRC(uint8_t * buffer, uint8_t length);


/**
  @brief Function to get the configure CAN resistor state 
  
  @param None
  @return enable 1 or disable 0
*/
uint8_t UserConfigTask_GetCANterminatorState(void);

/**
  @brief Function to set the configure CAN resistor state 
  
  @param enable 1 or disable 0
  @return none
*/

void UserConfigTask_UpdateCANterminatorState(uint8_t resistorState);

#endif
