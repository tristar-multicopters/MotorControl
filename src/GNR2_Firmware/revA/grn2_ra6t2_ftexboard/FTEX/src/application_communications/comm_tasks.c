/**
  * @file    comm_tasks.c
  * @brief   This module gather
  *
  */
#include "comm_tasks.h"
#include "mc_tasks.h"
#include "vc_tasks.h"

#include "comm_config.h"
#include "vc_config.h"

#include "gnr_main.h"

// CANOpen includes
#include "co_core.h"
#include "co_gnr2_specs.h"
// CAN logger
#include "can_logger.h"
#include "can_iot_comm.h"

#include "lcd_apt_comm.h"
// Serial Flash storage
//#include "serial_flash_storage.h"

/************* DEFINES ****************/

#define CAN_LOG_INTERVAL_TICK               25      /* CAN logger task send a new log frame every 25 RTOS ticks */
#define MAX_NUMBER_MISSED_HEARTBEAT         2       /* Max number of missed CANopen heartbeat from outside ganrunner device.
                                                        Above that, communication is considered lost. */


/********* PUBLIC MEMBERS *************/

uint16_t hCommErrors = COMM_NO_ERROR;  /* This global variable holds all error flags related to communications.
                                            Should be replaced later by a more elaborate structure. */

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


/** @brief  Callback function used for updating the values of the GNR
*           object dictionary.
*/
static void UpdateObjectDictionnary(void *p_arg)
{
#if GNR_IOT
	
	  CO_NODE  *pNode;
    pNode = (CO_NODE *)p_arg;
    
	  VCI_Handle_t * pVCI = &VCInterfaceHandle;
    // Get Bike Parameters 
    uint8_t hSpeed          = CanIot_GetSpeed(pVCI);
    uint16_t hPWR           = CanIot_GetPower(pVCI);
    uint8_t bSOC            = CanIot_GetSOC(pVCI);
    uint8_t bPAS            = CanIot_GetPAS(pVCI);
    uint8_t bMaxPas         = CanIot_GetMaxPAS();  
    uint16_t hMaxPwr        = CanIot_GetMaxPWR();
    uint16_t hErrorState    = CanIot_GetCurrentFaults(pVCI);
    uint16_t hFwVersion     = CanIot_GetFwVersion();
    // Get Serial Number
    uint64_t fSerialNbLow   = 0U;
    uint64_t fSrialNbHigh   = 0U;
    
    // Set Bike Parameters
    CanIot_SetPAS(pVCI,bPAS);

    if (pNode == NULL) 
    {        
        return;
    }

	if(CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL)
    {   
        /* Write commands in CANOpen object dictionnary received by SDO */
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_MEASURE, M1)), pNode, &hSpeed, CO_BYTE, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_POWER_MEASURE, M1)), pNode, &hPWR, CO_WORD, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SOC, M1)), pNode, &bSOC, CO_BYTE, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)), pNode, &bPAS, CO_BYTE, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_PAS, M1)), pNode, &bMaxPas, CO_BYTE, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_POWER, M1)), pNode, &hMaxPwr, CO_WORD, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_ERR_STATE, M1)), pNode, &hErrorState, CO_WORD, 0);
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SERIAL_NB, M2)), pNode, &fSerialNbLow, CO_LONG, 0); 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SERIAL_NB, M1)), pNode, &fSrialNbHigh, CO_LONG, 0); 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FW_VERSION, M1)), pNode, &hFwVersion, CO_WORD, 0);        
        /* Read commands in CANOpen object dictionnary received by SDO */
        //COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)), pNode, &bPAS, CO_BYTE, 0);
    }
#else
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
    #if !GNR_MASTER
    int16_t hMotor2TorqRef      = 0;
    uint8_t bMotor2Start        = 0;
    uint8_t bMotor2FaultAck     = 0;
    #endif

    if(CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL)
    {
        #if GNR_MASTER
        /* Check if no heartbeat was missed from slave  */
        if (CONmtGetHbEvents(&pNode->Nmt, GNR2_SLAVE_NODE_ID) <= MAX_NUMBER_MISSED_HEARTBEAT)
        {
            /* Update M1 feedback data to CANOpen object dictionnary */
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M1)), pNode, &hMotorSpeedMeas, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M1)), pNode, &hBusVoltage, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M1)), pNode, &hMotorTemp, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M1)), pNode, &hHeatsinkTemp, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M1)), pNode, &hMotorState, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M1)), pNode, &hMotorOccuredFaults, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M1)), pNode, &hMotorCurrentFaults, CO_WORD, 0);

            /* Update virtual motor 2 structure used by vehicle control layer */
            SlaveMCInterface_UpdateFeedback(&SlaveM2);
        }
        else
        {
            // Slave not present anymore, trigger vehicle communication fault
            hCommErrors |= MASTER_SLAVE_NO_HEARTBEAT;
            VCSTM_FaultProcessing(VCInterfaceHandle.pStateMachine, VC_SLAVE_COMM_ERROR, 0);
        }
        #else
        /* Check if no heartbeat was missed from master  */
        if (CONmtGetHbEvents(&pNode->Nmt, GNR2_MASTER_NODE_ID) <= MAX_NUMBER_MISSED_HEARTBEAT)
        {
            /* Update M2 feedback data to CANOpen object dictionnary */
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M2)), pNode, &hMotorSpeedMeas, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M2)), pNode, &hBusVoltage, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M2)), pNode, &hMotorTemp, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M2)), pNode, &hHeatsinkTemp, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M2)), pNode, &hMotorState, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M2)), pNode, &hMotorOccuredFaults, CO_WORD, 0);
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M2)), pNode, &hMotorCurrentFaults, CO_WORD, 0);

            /* Read commands in CANOpen object dictionnary received by RPDO */
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M2)), pNode, &hMotor2TorqRef, CO_WORD, 0);
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_START, M2)), pNode, &bMotor2Start, CO_BYTE, 0);
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, CO_BYTE, 0);

            /* Execute received commands using motor control api */
            MCInterface_ExecTorqueRamp(&MCInterface[0], hMotor2TorqRef);
            bMotor2Start ? MCInterface_StartMotor(&MCInterface[0]) : MCInterface_StopMotor(&MCInterface[0]);
            if (bMotor2FaultAck)
            {
                // Reset fault ack after reception
                bMotor2FaultAck = 0;
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, CO_BYTE, 0);
                MCInterface_FaultAcknowledged(&MCInterface[0]);
            }
        }
        else
        {
            // Master not present anymore, stop motor
            hCommErrors |= MASTER_SLAVE_NO_HEARTBEAT;
            MCInterface_StopMotor(&MCInterface[0]);
        }
        #endif
    }
#endif
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
#if GNR_IOT   
#else
    /* Initialize motor 2 handle to be used by vehicle control layer */
    SlaveMotorRegisterAddr_t M2RegAddr =
    {
        .wRegAddrMotorSpeed = CO_DEV(CO_OD_REG_MOTOR_SPEED, M2),
        .wRegAddrBusVoltage = CO_DEV(CO_OD_REG_BUS_VOLTAGE, M2),
        .wRegAddrCurrentFaults = CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M2),
        .wRegAddrOccuredFaults = CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M2),
        .wRegAddrState = CO_DEV(CO_OD_REG_MOTOR_STATE, M2),
        .wRegAddrMotorTemp = CO_DEV(CO_OD_REG_MOTOR_TEMP, M2),
        .wRegAddrHeatsinkTemp = CO_DEV(CO_OD_REG_HEATSINK_TEMP, M2),
        .wRegAddrStartMotor = CO_DEV(CO_OD_REG_MOTOR_START, M2),
        .wRegAddrFaultAck = CO_DEV(CO_OD_REG_FAULT_ACK, M2),
        .wRegAddrTorqueRef = CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M2),       
    };
    SlaveMCInterface_Init(&SlaveM2, &CONodeGNR, M2RegAddr);
#endif
    /* Select UART protocol */
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
                LCD_APT_Task(&LCD_APT_handle); //Run the APT task
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

    // Initialize canbus hardware layer and the CANopen stack
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

/**
  Task to handle the received messages anad to send messages through the CAN bus
*/
/*__NO_RETURN void Memory_Task (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
   
	EFlash_Storage_Handle_t * pEFComm = &EFlash_Storage_Handle;
	SF_Storage_Init(pEFComm);
    while(true)
    {
        osDelay(1);
    }
}*/