/**
  ******************************************************************************
  * @file    gnr_main.c
  * @author  FTEX inc
  * @brief   This file is the main application of the ganrunner motor controller firmware
  *
	******************************************************************************
	*/

#include "gnr_main.h"

#define TASK_0_TICK 	20
#define TASK_1_TICK 	40
#define TASK_2_TICK		80
#define TASK_3_TICK		160

osThreadId_t TSK_0_handle;
osThreadId_t TSK_1_handle;
osThreadId_t TSK_2_handle;
osThreadId_t TSK_3_handle;

static const osThreadAttr_t ThAtt_TSK_0 = {
	.name = "TSK_0",
	.stack_size = 512,
	.priority = osPriorityAboveNormal2
};

static const osThreadAttr_t ThAtt_TSK_1 = {
	.name = "TSK_1",
	.stack_size = 512,
	.priority = osPriorityAboveNormal1
};

static const osThreadAttr_t ThAtt_TSK_2 = {
	.name = "TSK_2",
	.stack_size = 512,
	.priority = osPriorityAboveNormal3,
};

static const osThreadAttr_t ThAtt_TSK_3 = {
	.name = "TSK_3",
	.stack_size = 512,
	.priority = osPriorityNormal2
};


void gnr_main(void)
{
	SystemCoreClockUpdate();
	
	osKernelInitialize();  // Initialise the kernel
	//EventRecorderInitialize(EventRecordAll,1U); // Initialise the events
	
	// Create task 0 
	TSK_0_handle      = osThreadNew(TSK_0, 
																	NULL,
																	&ThAtt_TSK_0);
	
	// Create task 1 
	TSK_1_handle      = osThreadNew(TSK_1, 
																	NULL,
																	&ThAtt_TSK_1);
	
	// Create task 2 
	TSK_0_handle      = osThreadNew(TSK_2, 
																	NULL,
																	&ThAtt_TSK_2);
	
	// Create task 3 
	TSK_1_handle      = osThreadNew(TSK_3, 
																	NULL,
																	&ThAtt_TSK_3);	
	
	// Start thread execution
	if (osKernelGetState() == osKernelReady)
	{
    osKernelStart();                    								
  }

}


__NO_RETURN void TSK_0 (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while(1)
	{
		xLastWakeTime += TASK_0_TICK;
		osDelayUntil(xLastWakeTime);	
	}
}
	
__NO_RETURN void TSK_1 (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while(1)
	{	
		xLastWakeTime += TASK_1_TICK;
		osDelayUntil(xLastWakeTime);	
	}
}	
	
__NO_RETURN void TSK_2 (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while(1)
	{		
		xLastWakeTime += TASK_2_TICK;
		osDelayUntil(xLastWakeTime);	
	}
}	

__NO_RETURN void TSK_3 (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	osDelay(250);
	uint32_t xLastWakeTime = osKernelGetTickCount();
	while(1)
	{			
		xLastWakeTime += TASK_3_TICK;
		osDelayUntil(xLastWakeTime);	
	}
}	


	