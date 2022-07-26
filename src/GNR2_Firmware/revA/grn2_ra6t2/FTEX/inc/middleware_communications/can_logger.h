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

#if ENABLE_CAN_LOGGER
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
	
}	VC_CAN_id_t;


// ==================== Public function prototypes ========================= //

/** @brief  Function for sending the vehicle status
    @p 		motorSelection  motor ID
*/
void CANLOG_getStatus(VCI_Handle_t * phandle, uint8_t motorSelection);

/** @brief  Function for sending the bus voltage value
*/
void CANLOG_getVbus(VCI_Handle_t * phandle);

/* Function for sending the reference and measured current values */
/** @brief  Function for sending the current values
    @p 		motorSelection  motor ID
*/
void CANLOG_getCurrent(VCI_Handle_t * phandle, uint8_t motorSelection);

/* Function for sending the motor reference and measured speed values*/
/** @brief  Function for sending the speed values
    @p 		motorSelection  motor ID
*/
void CANLOG_getSpeed(VCI_Handle_t * phandle, uint8_t motorSelection);

/* Function for sending the motor drive temperature */
/** @brief  Function for sending the motor drive temperature
    @p 		motorSelection  motor ID
*/
void CANLOG_SendTemperature(VCI_Handle_t * phandle, uint8_t motorSelection);

/* Function for sending the vehicle throttle value and the brake status*/
/** @brief  Function for sending the throttle value and the brake status
*/
void CANLOG_SendThrottleBrake(VCI_Handle_t * phandle);

#endif

#endif
