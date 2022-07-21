/**
  * @file    comm_tasks.c
  * @brief   This module gather 
  *
  */
#include "comm_tasks.h"
#include "vc_config.h"
#include "comm_config.h"

#include "gnr_main.h"


/************* DEBUG ****************/



/************* DEFINES ****************/



/************* TASKS ****************/


/**
  * @brief  It initializes the vehicle control application. Needs to be called before using
	*					vehicle control related modules.
  * @retval None
  */
void Comm_BootUp(void)
{	
    
    
    switch(UART0_handle.UARTProtocol)
	{	
		/*case UART_BAFANG:
			LCD_BAF_init(&VCInterfaceHandle);
			break;
		*/		
		case UART_APT:
			LCD_APT_init(&LCD_APT_handle, &VCInterfaceHandle, &UART0_handle);
			break;
		case UART_DISABLE:
        default:
			//Dont initialise the euart		
			break;
	}
}

__NO_RETURN void ProcessUARTFrames (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
		
	while(true)
	{
		osThreadFlagsWait(UART_FLAG, osFlagsWaitAny, osWaitForever);
		
        switch(UART0_handle.UARTProtocol)
		{			
			/*case UART_BAFANG:
				LCD_BAF_frame_Process();
				break;*/
			
			case UART_APT:
				LCD_APT_frame_Process(&LCD_APT_handle);
				break;
			default:
				break;
		}
	}
}
