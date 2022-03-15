/**
  ******************************************************************************
  * @file    main.c
  * @author  Sami Bouzid, FTEX
  * @brief   This file is the main application of the motor drive - Master
  *
	******************************************************************************
	*/


#include "md_comm.h"
#include "host_comm.h"

#include "nrf_delay.h"

#include "throttle.h"
#include "vc_tasks.h"

#define COMM_STACK_SIZE 256

extern TaskHandle_t TSK_MDcomm_handle;
extern TaskHandle_t TSK_FastLoopMD_handle;
extern TaskHandle_t TSK_SlowLoopMD_handle;
extern TaskHandle_t TSK_ReplyToHost_handle;
extern TaskHandle_t TSK_usbd_handle;
extern TaskHandle_t TSK_CANmsgTX_handle_t;

/**
 * @brief Function for main application entry.
 */
int main(void)
{	
	//nrf_delay_ms(100);
//	
//	VC_BootUp();
//	
//	/* Create task for frame communication with motor drive with priority set to 2 */
//	UNUSED_VARIABLE(xTaskCreate(
//															TSK_MDcomm,
//															"TSK_MDcomm",
//															configMINIMAL_STACK_SIZE+600,
//															NULL,
//															5,
//															&TSK_MDcomm_handle));							
//															
//	/* Create task to manage throttle input with priority set to 4 */
//	UNUSED_VARIABLE(xTaskCreate(TSK_FastLoopMD,
//															"TSK_FastLoopMD", 
//															configMINIMAL_STACK_SIZE+100, 
//															NULL, 
//															3, 
//															&TSK_FastLoopMD_handle));
//															
//  /* Create task for periodical communication with STM32 with priority set to 5 */
//	UNUSED_VARIABLE(xTaskCreate(TSK_SlowLoopMD, 
//															"TSK_SlowLoopMD", 
//															configMINIMAL_STACK_SIZE+100, 
//															NULL, 
//															2, 
//															&TSK_SlowLoopMD_handle));
//															
//	/* Create task for USBD Communication between host and nRF52 with priority set to 3*/
//	UNUSED_VARIABLE(xTaskCreate(TSK_init_usbd,
//															"TSK_USBD",
//															COMM_STACK_SIZE, 
//															NULL, 
//															5,
//															&TSK_usbd_handle));
//	
//  /* Create task to reply to the host with priority set to 2*/
//	UNUSED_VARIABLE(xTaskCreate(TSK_ReplyToHost,
//															"TSK_ReplyToHost",
//															COMM_STACK_SIZE, 
//															NULL, 
//															3,
//															&TSK_ReplyToHost_handle));

//	/* Create task to manage CANBUS transfer messages*/
//	UNUSED_VARIABLE(xTaskCreate(TSK_CANmsg,
//															"TSK_CAN",
//															COMM_STACK_SIZE, 
//															NULL, 
//															2,
//															&TSK_CANmsgTX_handle_t));	
															
	
	/* Start FreeRTOS scheduler. */
	vTaskStartScheduler();

	while (true)
	{
		// We should never get here...
	}
		
}


/** @} */
