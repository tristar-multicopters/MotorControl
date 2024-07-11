/**
  ******************************************************************************
  * @file     .h
  * @author   , FTEX
  * @brief   High level module that describes 
  *
  ******************************************************************************
*/
    
#ifndef __CAN_VEHICLE_INTERFACE_H
#define __CAN_VEHICLE_INTERFACE_H

#include "vc_interface.h"
#include "co_can_ra6t2.h"
#include "vc_parameters.h"
#include "battery_monitoring.h"
#include "pedal_assist.h"
#include "vc_constants.h"

// =========================== Defines ==================================== //
/* Defines */
#define MAX_POWER              1200                        // Max power value in W
#define SERIAL_NUMBER          "1234FTEX"                  // Dummy serial number
#define HW_VERSION             0x0104U                     // Dummy hardware version - v1.4
#define FW_VERSION             0x0405U                     // Dummy firmware version - v1.5


/* Errors to send by CanOpen */
#define CONTROLLER_OVER_T_FAULT     0x0001
#define MOTOR_START_U_FAULT         0x0002
#define MOTOR_OVER_C_FAULT          0x0004
#define HALL_SENSOR_FAULT           0x0008
#define WHEEL_SPEED_FAULT           0x0010
#define CONTROL_OVER_V              0x0020
#define CONTROL_UNDER_V             0x0040
#define CONTROL_OVER_T              0x0080
#define THROTTLE_FAULT              0x0100
#define PAS_FAULT                   0x0200


/* Other definitions */
#define BYTE_1_INDEX                1      /**< Index of Byte 1 of the data frame in a CAN message */
#define BYTE_4_INDEX                4      /**< Index of Byte 4 of the data frame in a CAN message */
#define MAX_DATA_IN_INIT            4      /**< Maxime data to send in a reply init                */
#define MAX_DATA_IN_SEG             8      /**< Maxime data to send in a reply segment             */
#define CURRENT_SENSOR_1            1      /**< Current sensor 1                                   */
#define CURRENT_SENSOR_2            2      /**< Current sensor 2                                   */
#define SENSOR_VALUES_BUFFER_SIZE   128    /**< Buffer size used for sensor current samples        */
#define DIGITAL_CURRENT_VALUE_MAX   65535  /**< Maximum possible value for digital current         */
#define ANALOG_CURRENT_VALUE_MAX    327.67 /**< Maximum possible value for analog current          */
#define AMPERE_TIMES_100            100    /**< Factor to multiply current value by 100            */

// ==================== Public function prototypes ========================= //

/**
  @brief  Get DE Power function
  @param  VCI_Handle_t handle
  @return DC power in uint16_t format
*/
uint16_t CanVehiInterface_GetVehicleDCPower(VCI_Handle_t * pHandle);

/**
  @brief  Get Torque function
  @param  VCI_Handle_t handle
  @return Torque in uint16_t format
*/
uint16_t CanVehiInterface_GetVehicleTorque(VCI_Handle_t * pHandle);

/**
  @brief  Get Power function
  @param  VCI_Handle_t handle
  @return Power in uint16_t format
*/
uint16_t CanVehiInterface_GetVehiclePower(VCI_Handle_t * pHandle);

/**
  @brief  Get State of Charge function
  @param  VCI_Handle_t handle
  @return State of Charge in uint8_t format
*/
uint8_t CanVehiInterface_GetVehicleSOC (VCI_Handle_t * pHandle);

/**
  @brief  Get PAS function
  @param  VCI_Handle_t handle
  @return PAS in uint8_t format
*/
uint8_t CanVehiInterface_GetVehiclePAS (VCI_Handle_t * pHandle);

/**
  @brief  Set PAS function
  @param  VCI_Handle_t handle
  @param  Set_PAS in uint8_t format
  @return None
*/
void CanVehiInterface_SetVehiclePAS (VCI_Handle_t * pHandle, uint8_t Set_PAS);

/**
  @brief  Get Maximum PAS function
  @param  None
  @return Maximum PAS in uint8_t format
*/
uint8_t CanVehiInterface_GetMaxVehiclePAS (void);

/**
  @brief  Get MAX DC Power function
  @param  None
  @return Maximum Power in uint16_t format
*/
uint16_t CanVehiInterface_GetMaxDCPWR (VCI_Handle_t * pHandle);

/**
  @brief  Get Current Faults function
  @param  VCI_Handle_t handle
  @return Get Current Faults in uint32_t format
*/
uint32_t CanVehiInterface_GetVehicleCurrentFaults (VCI_Handle_t * pHandle);

/**
  @brief  Get Current Break Status
  @param  VCI_Handle_t handle
  @return Get Current Brake Status in uint8_t format
*/
uint8_t CanVehiInterface_GetBrakeStatus (VCI_Handle_t * pHandle);

/**
  @brief  Get FW Version function
  @param  None
  @return Firmware Version in uint16_t format
*/
uint16_t CanVehiInterface_GetVehicleFwVersion(void);

/**
  @brief  Get HW Version function
  @param  None
  @return Hardware Version in uint16_t format
*/
uint16_t CanVehiInterface_GetVehicleHwVersion(void);

/**
  @brief  Get Serial Number function
  @param  None
  @return Serial Number in uint8_t format
*/
uint8_t CanVehiInterface_GetVehicleSerialNumber(void);

/**
  @brief  Get the wheel diamater used to calculate speed
  @param  void
  @return Diameter in inches
*/
uint8_t CanVehiInterface_GetWheelDiameter(void);

/**
  @brief  Update wheel diameter used to calculate speed
  @param  New value of the wheel diameter in inches
  @return None
*/
void CanVehiInterface_UpdateWheelDiameter(uint8_t aValue);

/**
  @brief  Get Speed function
  @param  VCI_Handle_t handle
  @return Speed in uint8_t format
*/
uint16_t CanVehiInterface_GetVehicleSpeed(void);

/**
  @brief  Get Speed Decimals function
  @param  VCI_Handle_t handle
  @return Speed Decimals in uint8_t format
*/
uint8_t CanVehiInterface_GetVehicleSpeedDec(void);

/**
  @brief  Get the current state of the front light 
  @param  pHandle: handle of the vehicle
  @return State of the light (0 Off, 1 On)
*/
uint8_t CanVehiInterface_GetFrontLightState(VCI_Handle_t * pHandle);

/**
  @brief  Set the current state of the front light 
  @param  pHandle: handle of the vehicle, desired state of the light (0 Off, 1 On)
  @return void
*/
void CanVehiInterface_ChangeFrontLightState(VCI_Handle_t * pHandle, uint8_t aState);

/**
  @brief  Get the current state of the rear light 
  @param  pHandle: handle of the vehicle
  @return State of the light (0 Off, 1 On)
*/
uint8_t CanVehiInterface_GetRearLightState(VCI_Handle_t * pHandle);

/**
  @brief  Set the current state of the rear light 
  @param  pHandle: handle of the vehicle, desired state of the light (0 Off, 1 On)
  @return void
*/
void CanVehiInterface_ChangeRearLightState(VCI_Handle_t * pHandle, uint8_t aState);

/**
  @brief  Getting the controller NTC temperature value 
  @param  pHandle: handle of the vehicle
  @return int16_t controller temperature
*/  
int16_t CanVehiInterface_GetControllerTemp(VCI_Handle_t * pHandle);

/**
  @brief  Getting the motor NTC temperature value 
  @param  pHandle: handle of the vehicle
  @return int16_t motor temperature
*/  
int16_t CanVehiInterface_GetMotorTemp(VCI_Handle_t * pHandle);

/**
  @brief Pass by reference all PAS values of the minimum torque percentage
  @param  pHandle: handle of the vehicle 
  @return none
 */
void CanVehiInterface_GetPasLevelMinTorque(VCI_Handle_t * pHandle, uint8_t * pasLevelMinTorque);

/**
  @brief Setup the vehicle to use a CAN screen
  @param  pHandle: handle of the vehicle 
  @return none
 */
void CanVehiInterface_SetupCANScreen(VCI_Handle_t * pHandle);

/**
  @brief Check if we are setup for a CAN screen
  @param none
  @return State of the CAN screen setup
 */
bool CanVehiInterface_CheckCANScreenSetup(void);
    
/**
  @brief Provide the vc layer with an updated Throttle value
  @param  pHandle: handle of the vehicle, uint16_t new value of throttle 
  @return none
 */
void CanVehiInterface_UpdateExternalThrottle(VCI_Handle_t * pHandle, uint16_t aNewThrottleVal);

/**
  @brief Enables the can layer to engage cruise control
  @param  pHandle: handle of the vehicle
  @return none
 */
void CanVehiInterface_EngageCruiseControl(VCI_Handle_t * pHandle);

/**
  @brief Enables the can layer to disengage cruise control
  @param  pHandle: handle of the vehicle
  @return none
 */
void CanVehiInterface_DisengageCruiseControl(VCI_Handle_t * pHandle);

/**
  @brief Gives the can layer acces to the current state of the cruise control
  @param  pHandle: handle of the vehicle
  @return sta eof the curise control 0 off, 1 on
 */
bool CanVehiInterface_GetCruiseControlState(VCI_Handle_t * pHandle);

/**
  @brief Set New PAS algorithm
  @param  pHandle: handle of the vehicle
  @return none
 */
void CanVehiInterface_SetAlgorithm(VCI_Handle_t * pHandle, PasAlgorithm_t aPASAlgo);

/**
  @brief Get bus voltage
  @param  pHandle: handle of the vehicle
  @return bus voltage
 */
uint16_t CanVehiInterface_GetBusVoltage();

/**
  @brief Get the RMS current read on a phase current sensor 
  @param  pHandle: handle of the vehicle
  @param  sensorNumber : Sensor selected, 1 for sensor 1, 2 for sensor 2
  @return Average RMS current read on sensor
 */
int16_t CanVehiculeInterface_GetSensorPhaseCurrentRMS(uint8_t sensorNumber);

/**
  @brief  Get the Odometer travelled distance
  @return odometer travelled distance
 */
uint32_t CanVehiInterface_GetOdometerDistance();

/**
  @brief  Get Pedal RPM function
  @param  VCI_Handle_t handle
  @return RPM in uint16_t format with one decimal place (return value/10) to convert
*/
uint16_t CanVehiInterface_GetVehiclePedalRPM();


/**
  @brief  Get Pedal torque value in percentage
  @param  VCI_Handle_t handle
  @return torque percentage in uint8_t format
*/
uint8_t CanVehiInterface_GetPedalTorqPercentage();


#endif /* __CAN_IOT_COMM_H */
