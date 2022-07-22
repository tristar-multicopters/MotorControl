/**
*  CAN_task.c
*  Module for defining and managing the CAN messages task
*/ 

#include "CAN_task.h"
#include "vc_config.h"

#include "co_core.h"
#include "co_gnr2_specs.h"

// ==================== Public Members ======================== //
/* Allocate a global CANopen node object */
CO_NODE GnR2Module;
// Semaphore for CANOpen timer
osSemaphoreId_t canTmrSemaphore;

// ==================== Private Members ======================== //
extern osThreadId_t CANThreadHandle;

// Attributes for the canTmrSemaphore
osSemaphoreAttr_t canTmrSemaphoreAttr =
{
  "CAN Tmr Semaphore",   						///< name of the semaphore
  0, 											///< attribute bits (none)
  NULL,       									///< memory for control block
  0   											///< size of provided memory for control block
};

// ==================== Private functions ======================== //
#if ENABLE_CAN_LOGGER
VC_CAN_id_t currenLogId = CAN_ID_STATUS_VC;

/** @brief  Function for sending the vehicle and motor diagnostics
            ENABLE_CAN_LOGGER flag has to be on to be able to log
*/
static void CANLOG_SendLogs(VCI_Handle_t * pVChandle);
#else
/** @brief  Callback function used for updating the values of the GNR2
*           object dictionary.
*/
static void GnR2ModuleApp(void *p_arg)
{
    CO_NODE  *node;
    node = (CO_NODE *)p_arg;
    
    CO_OBJ *objSpeed = CODictFind(&node->Dict, CO_DEV(0x2000, 0));
    uint8_t speed = 100;
    
    if(node == 0)
    {
        return;
    }
    
    if(CONmtGetMode(&node->Nmt) == CO_OPERATIONAL)
    {
        COObjWrValue(objSpeed, node,&speed,1,0);
    }
}
#endif
/******************************************************/

/**
  Task to handle the received messages and to send messages through the CAN bus
*/
__NO_RETURN void processCANmsgTask (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    #if ENABLE_CAN_LOGGER
    CAN_initInterface();
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    uint32_t xLastWakeTime = osKernelGetTickCount();
    #else
    uint32_t ticks;
    // Initialize canTmrSemaphore
    canTmrSemaphore = osSemaphoreNew(16, 0, &canTmrSemaphoreAttr);
    // Initialize hardware layer and the CANopen stack. 
	CONodeInit(&GnR2Module, &GnR2ModuleSpec);
	
    // Stop execution if an error is detected.
    if (CONodeGetErr(&GnR2Module) != CO_ERR_NONE) 
    {
			while(1);
	}
    
    /* Use CANopen software timer to create a cyclic function
	 * call to the callback function 'GnR2ModuleApp()' with a period
	 * of 1s (equal: 1000ms).
	 */
	ticks = COTmrGetTicks(&GnR2Module.Tmr, 1000U, (uint32_t)CO_TMR_UNIT_1MS);
	COTmrCreate(&GnR2Module.Tmr, 0, ticks, GnR2ModuleApp, &GnR2Module);
        
    /* Start the CANopen node and set it automatically to
	 * NMT mode: 'OPERATIONAL'.
	 */
	CONodeStart(&GnR2Module);
	CONmtSetMode(&GnR2Module.Nmt, CO_OPERATIONAL);
    #endif
    
    while(true)
    {
        // Send CAN log messages
        #if ENABLE_CAN_LOGGER
        CANLOG_SendLogs(pVCI); // Initialise the CAN interface
        xLastWakeTime += TASK_CAN_SAMPLE_TIME_TICK;
        osDelayUntil(xLastWakeTime);  
        #else
        COTmrProcess(&GnR2Module.Tmr);
        osSemaphoreAcquire(canTmrSemaphore, osWaitForever);
        #endif              
    }
}

#if ENABLE_CAN_LOGGER
/**
* Function for sending the vehicle and motor diagnostics
*/
static void CANLOG_SendLogs(VCI_Handle_t * pVChandle)
{
    switch(currenLogId)
    {
        case CAN_ID_STATUS_VC:
            CANLOG_getStatus(pVChandle, M_NONE);
            currenLogId = CAN_ID_THROTTLE_BRAKE;
        break;
        
        case CAN_ID_THROTTLE_BRAKE:
            CANLOG_SendThrottleBrake(pVChandle);
            currenLogId = CAN_ID_VBUS;
        break;
        
        case CAN_ID_VBUS:
            CANLOG_getVbus(pVChandle);
            currenLogId = CAN_ID_STATUS_M1;
        break;
        
        case CAN_ID_STATUS_M1:
            CANLOG_getStatus(pVChandle, M1);
            currenLogId = CAN_ID_CURRENT_M1;
        break;
        
        case CAN_ID_CURRENT_M1:
            CANLOG_getCurrent(pVChandle, M1);
            currenLogId = CAN_ID_SPEED_M1;
        break;
        
        case CAN_ID_SPEED_M1:
            CANLOG_getSpeed(pVChandle, M1);
            currenLogId = CAN_ID_TEMPERATURE_M1;
        break;
        
        case CAN_ID_TEMPERATURE_M1:
            CANLOG_SendTemperature(pVChandle, M1);
            currenLogId = CAN_ID_STATUS_VC;
        break;
        
        default:
            break;  
    }
}
#endif