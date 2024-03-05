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
#define BYTE_1_INDEX         1      /**< Index of Byte 1 of the data frame in a CAN message */
#define BYTE_4_INDEX         4      /**< Index of Byte 4 of the data frame in a CAN message */
#define MAX_DATA_IN_INIT     4      /**< Maxime data to send in a reply init                */
#define MAX_DATA_IN_SEG      8      /**< Maxime data to send in a reply segment             */

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
  @brief  Get PAS Algorithm function
  @param  VCI_Handle_t handle
  @return PAS algorithm, in uint8_t format
*/
uint8_t CanVehiInterface_GetVehiclePASAlgorithm (VCI_Handle_t * pHandle);

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
uint16_t CanVehiInterface_GetVehicleSpeed(VCI_Handle_t * pHandle);

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

#endif /* __CAN_IOT_COMM_H */
