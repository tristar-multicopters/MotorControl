/**
  * @file    comm_tasks.c
  * @brief   This module gather 
  *
  */
#include "comm_tasks.h"
#include "vc_config.h"
#include "comm_config.h"
#include "gnr_main.h"
// CANOpen includes
#include "co_core.h"
#include "co_gnr2_specs.h"

/************* DEBUG ****************/



/************* DEFINES ****************/

/********* PUBLIC MEMBERS *************/
// Semaphore for CANOpen timer
osSemaphoreId_t canTmrSemaphore;
#if !ENABLE_CAN_LOGGER


/********* PRIVATE MEMBERS *************/

extern osThreadId_t CANmanagerHandle;
// Attributes for the canTmrSemaphore
osSemaphoreAttr_t canTmrSemaphoreAttr =
{
  "CAN Tmr Semaphore",   						///< name of the semaphore
  0, 											///< attribute bits (none)
  NULL,       									///< memory for control block
  0   											///< size of provided memory for control block
};


/********* PRIVATE FUNCTIONS *************/

/** @brief  Callback function used for updating the values of the GNR2
*           object dictionary.
*/
static void GnR2ModuleApp(void *p_arg)
{
    CO_NODE  *node;
    node = (CO_NODE *)p_arg;
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    CO_OBJ *objSpeed    = CODictFind(&node->Dict, CO_DEV(0x2000, 0));
    CO_OBJ *objpwr      = CODictFind(&node->Dict, CO_DEV(0x2001, 0));
    CO_OBJ *objSOC      = CODictFind(&node->Dict, CO_DEV(0x2002, 0));
    CO_OBJ *objPAS      = CODictFind(&node->Dict, CO_DEV(0x2003, 0));
    CO_OBJ *objMaxPAS   = CODictFind(&node->Dict, CO_DEV(0x2004, 0));
    CO_OBJ *objMaxPwr   = CODictFind(&node->Dict, CO_DEV(0x2005, 0));
    CO_OBJ *objErrorSt  = CODictFind(&node->Dict, CO_DEV(0x2006, 0));
    CO_OBJ *objSerialNbHigh  = CODictFind(&node->Dict, CO_DEV(0x2007, 0));
    CO_OBJ *objSerialNbLow = CODictFind(&node->Dict, CO_DEV(0x2007, 1));
    CO_OBJ *objFWver    = CODictFind(&node->Dict, CO_DEV(0x2008, 0));
    
    int16_t speed     = MDI_GetAvrgMecSpeedUnit(pVCI->pPowertrain->pMDI,M1);
    int16_t pwr       = 750; // TODO: Implement function that allows to get the obtained power. Use a hardcoded value for now...
    uint8_t soc       = 75;  // TODO: Implement function that allows to get the SOC. Use a hardcoded value for now...
    uint8_t pas       = (uint8_t)VCI_ReadRegister(pVCI,REG_PAS_LEVEL);
    uint8_t maxPas    = (uint8_t)VCI_ReadRegister(pVCI,REG_PAS_MAXLEVEL);  
    uint16_t maxPwr   = 2000; // TODO: Implement function that allows to get the maxPower. Use a hardcoded value for now...
    uint16_t ErrorState = MDI_GetCurrentFaults(pVCI->pPowertrain->pMDI,M1);
    uint16_t FWVer    =  (uint16_t)VCI_ReadRegister(pVCI,REG_FIRMVER);
    // Get serial number
    uint64_t serialNbLow  =  (uint32_t)VCI_ReadRegister(pVCI,REG_DEVICE_ID_LOW);
    uint64_t serialNbHigh =  (uint64_t)VCI_ReadRegister(pVCI,REG_DEVICE_ID_HIGH);
    
    if(node == 0)
    {
        return;
    }
    
    if(CONmtGetMode(&node->Nmt) == CO_OPERATIONAL)
    {
        COObjWrValue(objSpeed, node,&speed,1,0);
        COObjWrValue(objpwr, node,&pwr,2,0);
        COObjWrValue(objSOC, node,&soc,1,0);
        COObjWrValue(objPAS, node,&pas,1,0);
        COObjWrValue(objMaxPAS, node,&maxPas,1,0);
        COObjWrValue(objMaxPwr, node,&maxPwr,2,0);
        COObjWrValue(objErrorSt, node,&ErrorState,2,0);
        COObjWrValue(objSerialNbLow, node,&serialNbLow,4,0); // Send only the Low part for now
        COObjWrValue(objSerialNbHigh, node,&serialNbHigh,4,0); // Send only the Low part for now
        COObjWrValue(objFWver, node,&FWVer,2,0);
    }
}
#endif

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

/**
  Task to handle the received messages and to send messages through the CAN bus
*/
__NO_RETURN void CANManagerTask (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    #if ENABLE_CAN_LOGGER
    CANo_DrvInit();
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    uint32_t xLastWakeTime = osKernelGetTickCount();
    #else
    uint32_t ticks;
    // Initialize canTmrSemaphore
    canTmrSemaphore = osSemaphoreNew(16, 0, &canTmrSemaphoreAttr);
    // Initialize hardware layer and the CANopen stack. 
	CONodeInit(&CAN_handle.canNode, &GnR2ModuleSpec);
	
    // Stop execution if an error is detected.
    if (CONodeGetErr(&CAN_handle.canNode) != CO_ERR_NONE) 
    {
			while(1);
	}
    
    /* Use CANopen software timer to create a cyclic function
	 * call to the callback function 'GnR2ModuleApp()' with a period
	 * of 1s (equal: 1000ms).
	 */
	ticks = COTmrGetTicks(&CAN_handle.canNode.Tmr, 1000U, (uint32_t)CO_TMR_UNIT_1MS);
	COTmrCreate(&CAN_handle.canNode.Tmr, 0, ticks, GnR2ModuleApp, &CAN_handle.canNode);
        
    /* Start the CANopen node and set it automatically to
	 * NMT mode: 'OPERATIONAL'.
	 */
	CONodeStart(&CAN_handle.canNode);
	CONmtSetMode(&CAN_handle.canNode.Nmt, CO_OPERATIONAL);
    #endif
    
    while(true)
    {
        // Send CAN log messages
        #if ENABLE_CAN_LOGGER
        CANLOG_SendLogs(pVCI); // Initialise the CAN interface
        xLastWakeTime += TASK_CAN_SAMPLE_TIME_TICK;
        osDelayUntil(xLastWakeTime);  
        #else
        COTmrProcess(&CAN_handle.canNode.Tmr);
        osSemaphoreAcquire(canTmrSemaphore, osWaitForever);
        
        #endif              
    }
}

/**
  Task to process the received messages from CANBus
*/
__NO_RETURN void CANProcessMsgs (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
		
	while(true)
	{
		osThreadFlagsWait(CAN_RX_FLAG, osFlagsWaitAny, osWaitForever);
        uCAL_CAN_ProcessRxMessage(&CAN_handle);
	}
}