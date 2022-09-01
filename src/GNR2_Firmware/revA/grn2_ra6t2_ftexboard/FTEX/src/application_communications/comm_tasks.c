/**
  * @file    comm_tasks.c
  * @brief   This module gather
  *
  */
#include "comm_tasks.h"
#include "mc_tasks.h"
#include "vc_tasks.h"

#include "comm_config.h"
#include "gnr_main.h"
// CANOpen includes
#include "co_core.h"
#include "co_gnr2_specs.h"
// CAN logger
#include "can_logger.h"


/************* DEFINES ****************/

#define CAN_LOG_INTERVAL_TICK               25      /* CAN logger task send a new log frame every 25 RTOS ticks */


/********* PUBLIC MEMBERS *************/
// Semaphore for CANOpen timer
osSemaphoreId_t canTmrSemaphore;

bool bCANOpenTaskBootUpCompleted = false;


/********* PRIVATE MEMBERS *************/

extern osThreadId_t CANOpenTaskHandle;
// Attributes for the canTmrSemaphore
osSemaphoreAttr_t canTmrSemaphoreAttr =
{
    "CAN_Tmr_Semaphore",   		//< name of the semaphore
    0, 							//< attribute bits (none)
    NULL,       				//< memory for control block
    0   						//< size of provided memory for control block
};

/********* PRIVATE FUNCTIONS *************/

/* The application specific SDO transfer finalization callback */
void AppCSdoUploadFinishCb(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{

}

/* The application specific SDO transfer finalization callback */
void AppCSdoDownloadFinishCb(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    (void)csdo;
    if (code == 0)
    {
        /* SDO completed succesfully */
        switch(index)
        {
            case CO_OD_REG_FAULT_ACK:
            {
            } break;

            default:
            {

            } break;
        }

    }
    else
    {
    /* a timeout or abort is detected during SDO transfer  */
    }
}


/** @brief  Callback function used for updating the values of the GNR
*           object dictionary.
*/
static void UpdateObjectDictionnary(void *p_arg)
{
    CO_NODE  *pNode;
    pNode = (CO_NODE *)p_arg;
    if (pNode == NULL) {return;}

    /* Get data from motor control and vehicle control layer */
    int16_t hMotorSpeedMeas         = MCInterface_GetAvrgMecSpeedUnit(&MCInterface[0]);
    int16_t hBusVoltage             = 0;
    int16_t hMotorTemp              = 0;
    int16_t hHeatsinkTemp           = 0;
    uint16_t hMotorState            = MCInterface_GetSTMState(&MCInterface[0]);
    uint16_t hMotorOccuredFaults    = MCInterface_GetOccurredFaults(&MCInterface[0]);
    uint16_t hMotorCurrentFaults    = MCInterface_GetCurrentFaults(&MCInterface[0]);
    #if GNR_MASTER
    int16_t hMotor1TorqRef      = MCInterface_GetTeref(&MCInterface[0]);
    uint8_t bMotor1Start        = MCInterface_GetSTMState(&MCInterface[0]) == M_RUN ? true : false;
    int16_t hMotor2TorqRef      = VCInterfaceHandle.pPowertrain->pMDI->VirtualMotor2.hTorqueRef;
    uint8_t bMotor2Start        = VCInterfaceHandle.pPowertrain->pMDI->VirtualMotor2.bStartMotor;
    uint8_t bMotor2FaultAck     = VCInterfaceHandle.pPowertrain->pMDI->VirtualMotor2.bFaultAck;
    #else
    int16_t hMotor2TorqRef      = 0;
    uint8_t bMotor2Start        = 0;
    uint8_t bMotor2FaultAck     = 0;
    #endif

    if(CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL)
    {
        #if GNR_MASTER
        /* Update M1 feedback data to CANOpen object dictionnary */
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M1)), pNode, &hMotorSpeedMeas, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M1)), pNode, &hBusVoltage, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M1)), pNode, &hMotorTemp, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M1)), pNode, &hHeatsinkTemp, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M1)), pNode, &hMotorState, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M1)), pNode, &hMotorOccuredFaults, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M1)), pNode, &hMotorCurrentFaults, 2, 0);

        /* Update M1 and M2 commands to CANOpen object dictionnary */
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M1)), pNode, &hMotor1TorqRef, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_START, M1)), pNode, &bMotor1Start, 1, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M2)), pNode, &hMotor2TorqRef, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_START, M2)), pNode, &bMotor2Start, 1, 0);

        /* Update virtual motor 2 structure used vehicle control layer */
        VirtualMotorFeedback_t VirtualMotor2Feedback = {0};
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M2)), pNode, &VirtualMotor2Feedback.bState, 2, 0);
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M2)), pNode, &VirtualMotor2Feedback.hOccuredFaults, 2, 0);
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M2)), pNode, &VirtualMotor2Feedback.hCurrentFaults, 2, 0);
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M2)), pNode, &VirtualMotor2Feedback.hMotorSpeed, 2, 0);
        MDI_UpdateVirtualMotorFeedback(VCInterfaceHandle.pPowertrain->pMDI, M2, VirtualMotor2Feedback);

        /* If vehicle control request a fault ack to motor 2, send a SDO */
        if (bMotor2FaultAck)
        {
            VCInterfaceHandle.pPowertrain->pMDI->VirtualMotor2.bFaultAck = false;

            CO_CSDO *csdo;
            csdo = COCSdoFind(&(CONodeGNR), 0);
            uint8_t Data = true;
            COCSdoRequestDownload(csdo, CO_DEV(CO_OD_REG_FAULT_ACK, M2), &Data, 1, AppCSdoDownloadFinishCb, 200);
        }

        #else
        /* Update M2 feedback data to CANOpen object dictionnary */
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M2)), pNode, &hMotorSpeedMeas, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M2)), pNode, &hBusVoltage, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M2)), pNode, &hMotorTemp, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M2)), pNode, &hHeatsinkTemp, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M2)), pNode, &hMotorState, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M2)), pNode, &hMotorOccuredFaults, 2, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M2)), pNode, &hMotorCurrentFaults, 2, 0);

        /* Read commands in CANOpen object dictionnary received by RPDO */
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M2)), pNode, &hMotor2TorqRef, 2, 0);
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_START, M2)), pNode, &bMotor2Start, 1, 0);
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, 1, 0);

        /* Execute received commands using motor control api */
        MCInterface_ExecTorqueRamp(&MCInterface[0], hMotor2TorqRef);
        bMotor2Start ? MCInterface_StartMotor(&MCInterface[0]) : MCInterface_StopMotor(&MCInterface[0]);
        if (bMotor2FaultAck)
        {
            // Reset fault ack after reception
            bMotor2FaultAck = 0;
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, 1, 0);
            MCInterface_FaultAcknowledged(&MCInterface[0]);
        }
        #endif
    }
}

/************* TASKS ****************/

/**
  * @brief  It initializes the vehicle control application. Needs to be called before using
	*		vehicle control related modules.
  * @retval None
  */
void Comm_BootUp(void)
{
    // Enable CAN transceiver by pulling standby_n and enable_n pin high
    struct GPIOConfig PinConfig;
    PinConfig.PinDirection = OUTPUT;
    PinConfig.PinPull      = NONE;
    PinConfig.PinOutput    = PUSH_PULL;
    uCAL_GPIO_ReInit(CAN_ENABLE_N_GPIO_PIN, PinConfig);
    uCAL_GPIO_Set(CAN_ENABLE_N_GPIO_PIN);

    PinConfig.PinDirection = OUTPUT;
    PinConfig.PinPull      = NONE;
    PinConfig.PinOutput    = PUSH_PULL;
    uCAL_GPIO_ReInit(CAN_STANDBY_N_GPIO_PIN, PinConfig);
    uCAL_GPIO_Set(CAN_STANDBY_N_GPIO_PIN);

    switch(UART0Handle.UARTProtocol)
	  {
        case UART_LOG_HS:
            LogHS_Init(&LogHS_handle, &VCInterfaceHandle, &UART0Handle);
            break;
    	case UART_APT:
            LCD_APT_init(&LCD_APT_handle, &VCInterfaceHandle, &UART0Handle);
    		break;
    	case UART_DISABLE:
            break;
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

    switch(UART0Handle.UARTProtocol)
        {
           case UART_APT:
                LCD_APT_frame_Process(&LCD_APT_handle);
                break;
           case UART_LOG_HS:
                LogHS_ProcessFrame(&LogHS_handle);
                break;
            default:
				break;
		}
	}
}


/**
  Task to manage CANOpen protocol
*/
__NO_RETURN void CANOpenTask (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    #if GNR_MASTER
    #else
    osDelay(100);
    #endif

    // Initialize canTmrSemaphore to control this task execution
    canTmrSemaphore = osSemaphoreNew(16, 0, &canTmrSemaphoreAttr);

    // Initialize hardware layer and the CANopen stack
	CONodeInit(&CONodeGNR, &GnR2ModuleSpec);

    // Stop execution if an error is detected on GNR2 node
    if (CONodeGetErr(&CONodeGNR) != CO_ERR_NONE)
    {
			while(1);
	}

    /* Use CANopen software timer to create a cyclic function
	 * call to the callback function 'UpdateObjectDictionnary()' with a period
	 * of 25ms.
	 */
    uint32_t ticks;
	ticks = COTmrGetTicks(&CONodeGNR.Tmr, 25U, (uint32_t)CO_TMR_UNIT_1MS);
	COTmrCreate(&CONodeGNR.Tmr, 0, ticks, UpdateObjectDictionnary, &CONodeGNR);

    /* Start the CANopen node and set it automatically to
	 * NMT mode: 'OPERATIONAL'.
	 */
	CONodeStart(&CONodeGNR);
	CONmtSetMode(&CONodeGNR.Nmt, CO_OPERATIONAL);

    bCANOpenTaskBootUpCompleted = true;

    while(true)
    {
        // This task unblocks when a CANOpen timer elapses
        osSemaphoreAcquire(canTmrSemaphore, osWaitForever);
        COTmrProcess(&CONodeGNR.Tmr);
    }
}


/**
  Task to handle the received messages anad to send messages through the CAN bus
*/
__NO_RETURN void CANLoggerTask (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    while(true)
    {
        if (bCANOpenTaskBootUpCompleted)
        {
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendStatus(&CoCanDriver, &VCInterfaceHandle, M_NONE); //Send vehicle status
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendStatus(&CoCanDriver, &VCInterfaceHandle, M1); //Send M1 status
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendStatus(&CoCanDriver, &VCInterfaceHandle, M2); //Send M2 status
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendVbus(&CoCanDriver, &VCInterfaceHandle); //Send Vbus
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendCurrent(&CoCanDriver, &VCInterfaceHandle, M1); //Send currents M1
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendCurrent(&CoCanDriver, &VCInterfaceHandle, M2); //Send currents M2
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendSpeed(&CoCanDriver, &VCInterfaceHandle, M1); //Send speed M1
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendSpeed(&CoCanDriver, &VCInterfaceHandle, M2); //Send speed M2
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendTemperature(&CoCanDriver, &VCInterfaceHandle, M1); //Send temperature M2
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendTemperature(&CoCanDriver, &VCInterfaceHandle, M2); //Send temperature M2
            osDelay(CAN_LOG_INTERVAL_TICK);
            CANLog_SendThrottleBrake(&CoCanDriver, &VCInterfaceHandle); //Send throttle & brake info
        }
    }
}
