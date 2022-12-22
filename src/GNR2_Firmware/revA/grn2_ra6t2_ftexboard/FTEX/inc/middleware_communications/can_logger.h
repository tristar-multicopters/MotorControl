/**
* @file   can_logger.h
* @author Jorge A Polo
* @brief  Middleware layer for managing CAN logging messages
*
* This module is used for building and sending CAN messages used for vehicle and
* motors diagnostics (logs)
*/

#ifndef CAN_LOGGER
#define CAN_LOGGER

#include "vc_interface.h"
#include "co_can_ra6t2.h"

typedef enum
{
	CAN_ID_STATUS_VC      = 0x1,
    CAN_ID_THROTTLE_BRAKE = 0x2,
	CAN_ID_VBUS			  = 0x10,
	
	CAN_ID_STATUS_M1 	  = 0xA0,
    CAN_ID_CURRENT_M1     = 0x20,
	CAN_ID_SPEED_M1		  = 0x30,
	CAN_ID_TEMPERATURE_M1 = 0x40,
	
	CAN_ID_STATUS_M2	  = 0xA1,
	CAN_ID_CURRENT_M2	  = 0x21,
	CAN_ID_SPEED_M2       = 0x31,
	CAN_ID_TEMPERATURE_M2 = 0x41,
	
}	VC_CAN_ID_t;


// ==================== Public function prototypes ========================= //

/** @brief  Function for sending the vehicle status
    @p 		bMotorSelection  motor ID
*/
void CANLog_SendStatus(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * phandle, uint8_t bMotorSelection);

/** @brief  Function for sending the bus voltage value
*/
void CANLog_SendVbus(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t* pHandle);

/* Function for sending the reference and measured current values */
/** @brief  Function for sending the current values
    @p 		bMotorSelection  motor ID
*/
void CANLog_SendCurrent(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * phandle, uint8_t bMotorSelection);

/* Function for sending the motor reference and measured speed values*/
/** @brief  Function for sending the speed values
    @p 		bMotorSelection  motor ID
*/
void CANLog_SendSpeed(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * phandle, uint8_t bMotorSelection);

/* Function for sending the motor drive temperature */
/** @brief  Function for sending the motor drive temperature
    @p 		bMotorSelection  motor ID
*/
void CANLog_SendTemperature(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * phandle, uint8_t bMotorSelection);

/* Function for sending the vehicle throttle value and the brake status*/
/** @brief  Function for sending the throttle value and the brake status
*/
void CANLog_SendThrottleBrake(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * phandle);


#endif
