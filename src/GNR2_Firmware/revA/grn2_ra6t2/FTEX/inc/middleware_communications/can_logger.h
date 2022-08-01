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

/* Function for sending all the vehicle and motor values */
/** @brief  This function should be called for getting 
*           vehicle and motor diagnostics
*   @param  pVChandle: pointer for vehicle handle
*/
void CANLOG_SendLogs(VCI_Handle_t * pVChandle);
#endif

#endif
