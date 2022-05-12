
#include "app_main.h"

#define TASK_0_TICK 	20
#define TASK_1_TICK 	40
#define TASK_2_TICK		80
#define TASK_3_TICK		160


typedef enum
{
	TASK_0 = 0,
	TASK_1,
	TASK_2,
	TASK_3
}Task_ID_t;

Task_ID_t Task_select;

bsp_io_level_t level = BSP_IO_LEVEL_LOW;



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


void application_main(void)
{
	
	SystemCoreClockUpdate();
	BootUp();
	
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

}	// End of application_main()


void BootUp(void)
{

	Init_Gpio();

}



__NO_RETURN void TSK_0 (void * pvParameter)
	{
		UNUSED_PARAMETER(pvParameter);
		osDelay(250);
		uint32_t xLastWakeTime = osKernelGetTickCount();
		while(1)
		{
			
			if(Task_select==TASK_0)
			{
				Toggle_Gpio();
			}
			
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
			if(Task_select==TASK_1)
			{
				Toggle_Gpio();
			}
			
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
			if(Task_select==TASK_2)
			{
				Toggle_Gpio();
			}
			
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
			if(Task_select==TASK_3)
			{
				Toggle_Gpio();
			}
			
			xLastWakeTime += TASK_3_TICK;
			osDelayUntil(xLastWakeTime);	
		}
	}	

	
void Init_Gpio(void)
{

	/* Initializes port to access pins. */
	R_IOPORT_Open(g_ioport.p_ctrl,g_ioport.p_cfg);

}	

void Toggle_Gpio(void)
{
	 /* Determine the next state of the Pin */
   if (BSP_IO_LEVEL_LOW == level)
	 {
     level = BSP_IO_LEVEL_HIGH;
   }
   else
   {
     level = BSP_IO_LEVEL_LOW;
   }
   /* Update Pin*/
   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_01, level);


}

	