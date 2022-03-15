/**
  ******************************************************************************
  * @file    main.c
  * @author  Sami Bouzid, FTEX
  * @brief   This file is the main application of the motor drive - Master
  *
	******************************************************************************
	*/

#include "nrf_delay.h"

#include "md_comm.h"
#include "throttle.h"
#include "vc_tasks.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include <cmsis_os2.h>
#include <rtx_os.h>
#include "EventRecorder.h"

extern osThreadId_t TSK_MDcomm_handle;
extern osThreadId_t TSK_FastLoopMD_handle;
extern osThreadId_t TSK_SlowLoopMD_handle;
extern osThreadId_t TSK_VehicleStateMachine_handle;
extern osThreadId_t TSK_eUART0_handle;
extern osThreadId_t TSK_CANmsgTX_handle_t;

//****************** THREAD ATTRIBUTES ******************//

static const osThreadAttr_t ThAtt_FastLoopMD = {
	.name = "TSK_Fast",
	.stack_size = 512,
	.priority = osPriorityAboveNormal2
};

static const osThreadAttr_t ThAtt_SlowLoopMD = {
	.name = "TSK_Slow",
	.stack_size = 512,
	.priority = osPriorityAboveNormal1
};

static const osThreadAttr_t ThAtt_VehicleStateMachine = {
	.name = "TSK_VehicleStateMachine",
	.stack_size = 512,
	.priority = osPriorityAboveNormal3,
};

static const osThreadAttr_t ThAtt_MDComm = {
	.name = "TSK_MDComm",
	.stack_size = 512,
	.priority = osPriorityNormal2
};

static const osThreadAttr_t ThAtt_eUART = {
	.name = "TSK_eUART",
	.stack_size = 512,
	.priority = osPriorityNormal3
};

#if HOSTCOMM_ENABLE
static const osThreadAttr_t ThAtt_HOSTComm = {
	.name = "TSK_HOSTComm",
	.stack_size = 512,
	.priority = osPriorityNormal3
};
#endif

#if CANBUS_ENABLE
static const osThreadAttr_t ThAtt_CANmsgTX = {
	.name = "TSK_CANbus",
	.stack_size = 512,
	.priority = osPriorityNormal1
};
#endif

/**************************************************************/

/**
 * @brief Function for main application entry.
 */
int main(void)
{	
	SystemCoreClockUpdate();
	VC_BootUp();
	
	osKernelInitialize();  // Initialise the kernel
	EventRecorderInitialize(EventRecordAll,1U); // Initialise the events
	
	// Create task to manage communication between nRF and STM 
	TSK_MDcomm_handle      = osThreadNew(TSK_MDcomm, 
																			NULL,
																			&ThAtt_MDComm);
	// Create task to manage vehicle state and throttle input 
	TSK_FastLoopMD_handle  = osThreadNew(TSK_FastLoopMD,
																			NULL, 
																			&ThAtt_FastLoopMD);
  // Create task to get regs values with low priority from STM 
	TSK_SlowLoopMD_handle  = osThreadNew(TSK_SlowLoopMD,
																			 NULL,
																			 &ThAtt_SlowLoopMD);
	TSK_VehicleStateMachine_handle = osThreadNew(TSK_VehicleStateMachine,
																			 NULL,
																			 &ThAtt_VehicleStateMachine);
	// Create task to manage UART0 communication 																		 
	TSK_eUART0_handle = osThreadNew(TSK_eUART,
																			 NULL,
																			 &ThAtt_eUART);

	#if HOSTCOMM_ENABLE
	/* Create task to manage communication between nRF and host (PC, mobile, ...) */
	TSK_HOSTcomm_handle      = osThreadNew(TSK_HOSTcomm, 
																			NULL,
																			&ThAtt_HOSTComm);
	#endif
	
	#if CANBUS_ENABLE
	/* Create task to manage CAN Protocol */																		 
	TSK_CANmsgTX_handle_t  = osThreadNew(TSK_CANmsg, 
																			 NULL, 
																			 &ThAtt_CANmsgTX);
	#endif
	
	// Start thread execution
	if (osKernelGetState() == osKernelReady)
	{
    osKernelStart();                    								
  }
	
	while (true)
	{
		// We should never get here...
	}
}


/** @} */
