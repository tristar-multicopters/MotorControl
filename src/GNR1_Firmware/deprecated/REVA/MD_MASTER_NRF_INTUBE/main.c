/**
  ******************************************************************************
  * @file    main.c
  * @author  Sami Bouzid, FTEX
  * @brief   This file is the main application of the motor drive - Master
  *
	******************************************************************************
	*/


#include "vc_tasks.h"

TaskHandle_t TSK_MDreceiveFrames_handle;
TaskHandle_t TSK_MDsendFrames_handle;
TaskHandle_t TSK_commManager_handle;
TaskHandle_t TSK_RCM_handle;
TaskHandle_t TSK_Throttle_handle;

/**
 * @brief Function for main application entry.
 */
int main(void)
{				
//	/* Create task for frame communication with motor drive with priority set to 2 */
//	UNUSED_VARIABLE(xTaskCreate(TSK_MDsendFrames, "TSK_MDsendFrames", configMINIMAL_STACK_SIZE+400, NULL, 2, &TSK_MDsendFrames_handle));
//	
//	/* Create task for frame communication with motor drive with priority set to 2 */
//	UNUSED_VARIABLE(xTaskCreate(TSK_MDreceiveFrames, "TSK_MDreceiveFrames", configMINIMAL_STACK_SIZE+400, NULL, 2, &TSK_MDreceiveFrames_handle));
	
	/* Create task to manage throttle input with priority set to 4 */
	UNUSED_VARIABLE(xTaskCreate(TSK_Throttle, "TSK_Throttle", configMINIMAL_STACK_SIZE+100, NULL, 4, &TSK_Throttle_handle));
		
		/* Create task to manage ADC regular acquisitions with priority set to 2 */
	UNUSED_VARIABLE(xTaskCreate(TSK_RCM, "TSK_RCM", configMINIMAL_STACK_SIZE+100, NULL, 2, &TSK_RCM_handle));
	
//	/* Create task to manage communications between systems with priority set to 4 */
//	UNUSED_VARIABLE(xTaskCreate(TSK_commManager, "TSK_commManager", configMINIMAL_STACK_SIZE+100, NULL, 4, &TSK_commManager_handle));
//	

	/* Start FreeRTOS scheduler. */
	vTaskStartScheduler();

	while (true)
	{
		// We should never get here...
	}
		
}

/** @} */
