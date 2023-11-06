/**
  * @file    comm_tasks.c
  * @brief   This module gather
  *
  */
#include "comm_tasks.h"
#include "mc_tasks.h"
#include "vc_tasks.h"
#include "vc_parameters.h"

#include "comm_config.h"
#include "vc_config.h"
#include "Utilities.h"
#include "gnr_main.h"

// CANOpen includes
#include "co_core.h"
#include "co_gnr2_specs.h"
// CAN logger
#include "can_logger.h"
#include "can_vehicle_interface.h"

// Serial Flash storage
#include "serial_flash_storage.h"

// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop

/************* DEFINES ****************/

#define CAN_LOG_INTERVAL_TICK               25      /* CAN logger task send a new log frame every 25 RTOS ticks */
#define MAX_NUMBER_MISSED_HEARTBEAT         2       /* Max number of missed CANopen heartbeat from outside ganrunner device.
                                                       Above that, communication is considered lost. */
#define VEHICLE_PARAM  0
#define CAN_PARAM      1 

/********* PUBLIC MEMBERS *************/

uint16_t hCommErrors = COMM_NO_ERROR;  /* This global variable holds all error flags related to communications.
                                            Should be replaced later by a more elaborate structure. */

bool bCANOpenTaskBootUpCompleted = false;


/********* PRIVATE MEMBERS *************/

extern osThreadId_t CANOpenTaskHandle;

/********* PRIVATE FUNCTIONS *************/


/** @brief  Callback function used for updating the values of the GNR
*           object dictionary.
*/
static void UpdateObjectDictionnary(void *p_arg)
{

	CO_NODE  *pNode;
    pNode = (CO_NODE *)p_arg;
    
	VCI_Handle_t * pVCI = &VCInterfaceHandle;
    // Get Bike Parameters 
    uint8_t  hSpeed[2]; 
    uint16_t hPWR[2];
    uint8_t  bSOC[2];
    uint8_t  bPAS[2];
    uint8_t  bPasAlgorithm[2];
    uint16_t hMaxPwr[2];
    uint8_t  hWheelDiameter[2];
    uint8_t  hFrontLightState[2];
    uint8_t  hRearLightState[2];
    uint16_t hErrorState[2];
    
    
    
    ASSERT(pVCI != NULL);
    ASSERT(pVCI->pPowertrain != NULL);
    ASSERT(pVCI->pPowertrain->pPAS != NULL);
    ASSERT(pVCI->pPowertrain->pPAS->pWSS != NULL);
    
    //only master can access theses parameters.
    if (VcAutodeter_GetGnrState())
    {
        hSpeed[VEHICLE_PARAM]  = (uint8_t)  CanVehiInterface_GetVehicleSpeed(pVCI);
        hPWR[VEHICLE_PARAM]    = (uint16_t) CanVehiInterface_GetVehiclePower(pVCI);
        bSOC[VEHICLE_PARAM]    = (uint8_t)  CanVehiInterface_GetVehicleSOC(pVCI);
        bPAS[VEHICLE_PARAM]    = (uint8_t)  CanVehiInterface_GetVehiclePAS(pVCI); 
        bPasAlgorithm[VEHICLE_PARAM] = (uint8_t) CanVehiInterface_GetVehiclePASAlgorithm(pVCI);
        hMaxPwr[VEHICLE_PARAM] = (uint16_t) CanVehiInterface_GetVehicleMaxPWR(pVCI);
        hFrontLightState[VEHICLE_PARAM] = CanVehiInterface_GetFrontLightState(pVCI);
        hRearLightState[VEHICLE_PARAM]  = CanVehiInterface_GetRearLightState(pVCI);
        hWheelDiameter[VEHICLE_PARAM]   = CanVehiInterface_GetWheelDiameter();
        hErrorState[VEHICLE_PARAM] = (uint16_t) CanVehiInterface_GetVehicleCurrentFaults(pVCI);
    }
    else
    {
        hErrorState[VEHICLE_PARAM] = 0x0000;
    }
    
    /***************Throttle/Pedal Assist variables******************************/
    uint8_t pasAlgorithm;
    uint8_t maxPAS;
    uint8_t pasMaxPower;
    uint8_t torqueMinimumThresholdStartup;    
    uint8_t startupTorqueMinimumThresholdSpeed;
    uint8_t torqueMinimumThreshold;
    uint8_t torqueSensorMultiplier;
    uint8_t torqueMaxSpeed;
    
    uint8_t cadenceLevelSpeed[10];
    uint8_t torqueLevelPower[10];
    uint8_t maxSpeed;
    uint8_t walkModeSpeed;
                                          
    /***********variable used to get the key code that enable user data configuration to be updated.**************/
    uint16_t keyUserDataConfig = 0;
                                          
    //variable used to verify if a firmware update command was received or not.
    uint8_t FirmwareUpdateCommand = 0;                                           
    
    //
    #if SUPPORT_SLAVE_ON_IOT | !GNR_IOT
    /* Get data from motor control and vehicle control layer */
    int16_t hMotorSpeedMeas         = MCInterface_GetAvrgMecSpeedUnit(&MCInterface[0]);
    int16_t hBusVoltage             = 0;
    int16_t hMotorTemp              = 0;
    int16_t hHeatsinkTemp           = 0;
    uint16_t hMotorState            = MCInterface_GetSTMState(&MCInterface[0]);
    uint32_t wMotorOccuredFaults    = MCInterface_GetOccurredFaults(&MCInterface[0]);
    uint32_t wMotorCurrentFaults    = MCInterface_GetCurrentFaults(&MCInterface[0]);
    #endif
    
    //theses variables are only used
    //when device is on slave configuration
    //This means IOT == 0
    #if !GNR_IOT
    int16_t hMotor2TorqRef      = 0;
    uint8_t bMotor2Start        = 0;
    uint8_t bMotor2FaultAck     = 0;
    #endif
    
    if (pNode == NULL) 
    {        
        return;
    }
    
    Comm_InitODWithUserConfig(pNode);
    
    #if GNR_IOT

	if(CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL)
    {                  
        /* Read commands in CANOpen object dictionnary received by SDO */
        //COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)), pNode, &bPAS, sizeof(uint8_t));
        #if SUPPORT_SLAVE_ON_IOT
        /* Check if no heartbeat was missed from slave  */
        if (!VCFaultManagment_MasterSlaveCommunicationLost())
        {
            /* Update M1 feedback data to CANOpen object dictionnary */
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M1)), pNode, &hMotorSpeedMeas, sizeof(hMotorSpeedMeas));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M1)), pNode, &hBusVoltage, sizeof(hBusVoltage));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M1)), pNode, &hMotorTemp, sizeof(hMotorTemp));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M1)), pNode, &hHeatsinkTemp, sizeof(hHeatsinkTemp));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M1)), pNode, &hMotorState, sizeof(hMotorState));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M1)), pNode, &wMotorOccuredFaults, sizeof(wMotorOccuredFaults));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M1)), pNode, &wMotorCurrentFaults, sizeof(wMotorCurrentFaults));
            
            /* Update virtual motor 2 structure used by vehicle control layer */
            SlaveMCInterface_UpdateFeedback(&SlaveM2);
            
            //clear heart beat flag error is was not cleared yet.
            if (hCommErrors & MASTER_SLAVE_NO_HEARTBEAT)
            {
                // Slave is present, clear vehicle communication fault
                hCommErrors &= ~MASTER_SLAVE_NO_HEARTBEAT;
            }
        }
        else
        {
            //verify if the error was already set or not.
            //if yes, doesn't set it again.
            if (!(hCommErrors & MASTER_SLAVE_NO_HEARTBEAT))
            {
                // Slave not present anymore, trigger vehicle communication fault
                hCommErrors |= MASTER_SLAVE_NO_HEARTBEAT;
                VCSTM_FaultProcessing(VCInterfaceHandle.pStateMachine, VC_SLAVE_COMM_ERROR, 0);
            }
        }
        #endif
    }
    #else

    if(CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL)
    {
        if (VcAutodeter_GetGnrState())
        {
            /* Check if no heartbeat was missed from slave  */
            if (!VCFaultManagment_MasterSlaveCommunicationLost())
            {
                /* Update M1 feedback data to CANOpen object dictionnary */
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M1)), pNode, &hMotorSpeedMeas, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M1)), pNode, &hBusVoltage, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M1)), pNode, &hMotorTemp, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M1)), pNode, &hHeatsinkTemp, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M1)), pNode, &hMotorState, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M1)), pNode, &wMotorOccuredFaults, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M1)), pNode, &wMotorCurrentFaults, sizeof(uint32_t));

                /* Update virtual motor 2 structure used by vehicle control layer */
                SlaveMCInterface_UpdateFeedback(&SlaveM2);
            
                //clear heart beat flag error is was not cleared yet.
                if (hCommErrors & MASTER_SLAVE_NO_HEARTBEAT)
                {
                    // Slave is present, clear vehicle communication fault
                    hCommErrors &= ~MASTER_SLAVE_NO_HEARTBEAT;
                }
            }
            else
            {
                //verify if the error was already set or not.
                //if yes, doesn't set it again.
                if (!(hCommErrors & MASTER_SLAVE_NO_HEARTBEAT))
                {
                    // Slave not present anymore, trigger vehicle communication fault
                    hCommErrors |= MASTER_SLAVE_NO_HEARTBEAT;
                    VCSTM_FaultProcessing(VCInterfaceHandle.pStateMachine, VC_SLAVE_COMM_ERROR, 0);
                }
            }
        }
        else
        {
            /* Check if no heartbeat was missed from master  */
            if (!VCFaultManagment_MasterSlaveCommunicationLost())
            {
                /* Update M2 feedback data to CANOpen object dictionnary */
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SPEED, M2)), pNode, &hMotorSpeedMeas, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M2)), pNode, &hBusVoltage, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP, M2)), pNode, &hMotorTemp, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M2)), pNode, &hHeatsinkTemp, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M2)), pNode, &hMotorState, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_OCC_FAULTS, M2)), pNode, &wMotorOccuredFaults, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_CUR_FAULTS, M2)), pNode, &wMotorCurrentFaults, sizeof(uint32_t));

                /* Read commands in CANOpen object dictionnary received by RPDO */
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M2)), pNode, &hMotor2TorqRef, sizeof(uint16_t));
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_START, M2)), pNode, &bMotor2Start, sizeof(uint8_t));
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, sizeof(uint8_t));
                
                /* Execute received commands using motor control api */
                MCInterface_ExecTorqueRamp(&MCInterface[0], hMotor2TorqRef);
                bMotor2Start ? MCInterface_StartMotor(&MCInterface[0]) : MCInterface_StopMotor(&MCInterface[0]);
                if (bMotor2FaultAck)
                {
                    // Reset fault ack after reception
                    bMotor2FaultAck = 0;
                    //critical section
                    COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, sizeof(uint8_t));
                    //end critical section
                    MCInterface_FaultAcknowledged(&MCInterface[0]);
                }
            
                //clear heart beat flag error is was not cleared yet.
                if (hCommErrors & MASTER_SLAVE_NO_HEARTBEAT)
                {
                    // Slave is present, clear vehicle communication fault
                    hCommErrors &= ~MASTER_SLAVE_NO_HEARTBEAT;
                }
            }
            else
            {
                //verify if the error was already set or not.
                //if yes, doesn't set it again.
                if (!(hCommErrors & MASTER_SLAVE_NO_HEARTBEAT))
                {
                    // Master not present anymore, stop motor
                    hCommErrors |= MASTER_SLAVE_NO_HEARTBEAT;
                    MCInterface_StopMotor(&MCInterface[0]);
                }
            }
        }
    }
    #endif

    //verify if the the node is operational to update the OD and if the device is a master
    //slave doesn't support the variables below.
    if((CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL) && (VcAutodeter_GetGnrState() == true))
    {
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_KEY_USER_DATA_CONFIG, 0)), pNode, &keyUserDataConfig, sizeof(uint16_t));
        
        //verifiy if a user parameter update was requested. if yes, the object dictionary, associated with measured variables,
        //will not be updated from the vc layer.
        if((keyUserDataConfig != KEY_USER_DATA_CONFIG_BEING_UPDATED) && (keyUserDataConfig != KEY_USER_DATA_CONFIG_UPDATED))
        {
            //Get the latest value of these parameters            
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_MEASURE, M1)), pNode, &hSpeed[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_POWER_MEASURE, M1)), pNode, &hPWR[CAN_PARAM], sizeof(uint16_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SOC, M1)),           pNode, &bSOC[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)),     pNode, &bPAS[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_ALGORITHM, 0)),  pNode, &bPasAlgorithm[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_POWER, M1)),     pNode, &hMaxPwr[CAN_PARAM], sizeof(uint16_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_ERR_STATE, M1)),     pNode, &hErrorState[CAN_PARAM], sizeof(uint16_t));
            
            
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS_DIAMETER, 0)),     pNode, &hWheelDiameter[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0)),     pNode, &hFrontLightState[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 0)),     pNode, &hRearLightState[CAN_PARAM], sizeof(uint8_t));
             
            // Check if there were changes made to a parameter by a can device
            
            if(hSpeed[VEHICLE_PARAM] != hSpeed[CAN_PARAM])
            {
                 hSpeed[CAN_PARAM] = hSpeed[VEHICLE_PARAM];
            }                
            
            if(hPWR[VEHICLE_PARAM] != hPWR[CAN_PARAM])
            {
                 hPWR[CAN_PARAM] = hPWR[VEHICLE_PARAM];
            }
            
            if(bSOC[VEHICLE_PARAM] != bSOC[CAN_PARAM])
            {
                 bSOC[CAN_PARAM] = bSOC[VEHICLE_PARAM];   
            }    
            bool WriteOBJDict = false;
            
            #if SCREEN_PROTOCOL == UART_APT   
                if((bPAS[VEHICLE_PARAM] != bPAS[CAN_PARAM]) || !LCD_APT_handle.APTStabilizing)
                {
                    WriteOBJDict = true; 
                    if (LCD_APT_handle.APTChangePasFlag) // Check if PAS was changed by non-can source                          
                    {    
                        bPAS[CAN_PARAM] = bPAS[VEHICLE_PARAM];  // if it was set the CAN pas as the vehicle pas                                    
                        LCD_APT_handle.APTChangePasFlag = false;
                    }
                    else // if it wasnt changed by a non-can then it was changed by a can source
                    {   
                        CanVehiInterface_SetVehiclePAS (&VCInterfaceHandle, bPAS[CAN_PARAM]);  // propagate the change in the vehicle
                    }
                }                               
            #elif SCREEN_PROTOCOL == UART_CLOUD_5S  
            
                if(bPAS[VEHICLE_PARAM] != bPAS[CAN_PARAM])
                {
                    WriteOBJDict = true; 
                    if (LCD_Cloud_5S_handle.cloud5SChangePasFlag == true) // Check if PAS was changed by non-can source                          
                    {    
                        bPAS[CAN_PARAM] = bPAS[VEHICLE_PARAM];  // if it was set the CAN pas as the vehicle pas                                    
                        LCD_Cloud_5S_handle.cloud5SChangePasFlag = false;
                    }
                    else // if it wasnt changed by a non-can then it was changed by a can source
                    {   
                        LCD_Cloud_5S_handle.isScreenSlave = true;
                        CanVehiInterface_SetVehiclePAS (&VCInterfaceHandle, bPAS[CAN_PARAM]);  // propagate the change in the vehicle
                    }
                }                    
            #endif

            
            if(bPasAlgorithm[VEHICLE_PARAM] != bPasAlgorithm[CAN_PARAM])
            {
               bPasAlgorithm[VEHICLE_PARAM] = bPasAlgorithm[CAN_PARAM];  
            }  
            
            
            if(hWheelDiameter[VEHICLE_PARAM] != hWheelDiameter[CAN_PARAM])
            {
                CanVehiInterface_UpdateWheelDiameter(hWheelDiameter[CAN_PARAM]);            
            }
           
            // If the light status in OBJ dict and vehicle don't match, update the one in the vehicle
            if(hFrontLightState[VEHICLE_PARAM] != hFrontLightState[CAN_PARAM])
            {
                // Keep this commented until CAN screens options has been added
                //CanVehiInterface_ChangeFrontLightState(&VCInterfaceHandle,hFrontLightState[CAN_PARAM]);            
            }
            if(hRearLightState[VEHICLE_PARAM] != hRearLightState[CAN_PARAM])
            {
                // Keep this commented until CAN screens options has been added
                //CanVehiInterface_ChangeRearLightState(&VCInterfaceHandle,hRearLightState[CAN_PARAM]);            
            }
            
            
            // Update the OD with the new PAS algorithm if needed
            if(hMaxPwr[VEHICLE_PARAM] != hMaxPwr[CAN_PARAM])
            {
                hMaxPwr[CAN_PARAM] = hMaxPwr[VEHICLE_PARAM];
            }

            if(hErrorState[VEHICLE_PARAM] != hErrorState[CAN_PARAM])
            {
                // Behavior TBD 
            }                
            
            /**************Write the repesctive OD ID, updating the OD that us read by the IOT module using SDO.*************/
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_MEASURE, M1)), pNode, &hSpeed[CAN_PARAM], sizeof(uint8_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_POWER_MEASURE, M1)), pNode, &hPWR[CAN_PARAM], sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SOC, M1)),           pNode, &bSOC[CAN_PARAM], sizeof(uint8_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SOC, M1)),           pNode, &bSOC[CAN_PARAM], sizeof(uint8_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_ALGORITHM, M1)), pNode, &bPasAlgorithm[CAN_PARAM], sizeof(uint8_t));
            
            if (WriteOBJDict)
            {    
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)),     pNode, &bPAS[CAN_PARAM], sizeof(uint8_t));
            }
            
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_POWER, M1)),     pNode, &hMaxPwr[CAN_PARAM], sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_ERR_STATE, M1)),     pNode, &hErrorState[CAN_PARAM], sizeof(uint16_t));
            
                    
            //Read the OD responsible to hold the firmware update command.
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 0)), pNode, &FirmwareUpdateCommand, sizeof(uint8_t));
            
            //check if the system was woke up by a firmware command update or not.
            PWREN_CheckFirmwareUpdateCommand(VCInterfaceHandle.pPowertrain->pPWREN, FirmwareUpdateCommand);
            
            //only master can send msgs to IOT module.
            //but not all bikes have IOT, so a third check if necessary
            //using the function Comm_CheckIotUsage. This fucntion return 
            //true if the selected model must have this IOT functionality.
            if (VcAutodeter_GetGnrState() && GNR_IOT && (Comm_CheckIotUsage()))
            {
                //fucntion used to inform IOT module that the GNR if on.
                PWREN_SetIotSystemIsOn(pNode, VCInterfaceHandle.pPowertrain->pPWREN);
            }
        }
        else
        {
            //verify is user data config is ready to be write in data flash memory.
             if(keyUserDataConfig == KEY_USER_DATA_CONFIG_UPDATED)
             {
                 /**********read value hold by the OD****************/
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_ALGORITHM, 0)), pNode, &pasAlgorithm, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_PAS, 0)), pNode, &maxPAS, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MAX_POWER, 0)), pNode, &pasMaxPower, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 0)), pNode, &torqueMinimumThreshold, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 1)), pNode, &torqueMinimumThresholdStartup, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 2)), pNode, &startupTorqueMinimumThresholdSpeed, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 0)), pNode, &torqueSensorMultiplier, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MAX_SPEED, 0)), pNode, &torqueMaxSpeed, sizeof(uint8_t));
        
                 for(uint8_t n = PAS_0;n <= PAS_9;n++)
                 {
                    COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CADENCE_HYBRID_LEVEL, n)), pNode, &cadenceLevelSpeed[n], sizeof(uint8_t));
                    COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_LEVEL_POWER, n)), pNode, &torqueLevelPower[n], sizeof(uint8_t));
                 }
        
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_SPEED, 0)), pNode, &maxSpeed, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 0)), pNode, &walkModeSpeed, sizeof(uint8_t));
            
                 /******update all variables used to keep the user data config that will be written in to the usaer data flash.****/
                 
                 //upadat Throttle/Pedal Assist variables that will be write into the user data flash.
                 UserConfigTask_UpdataPasAlgorithm(pasAlgorithm);
                 UserConfigTask_UpdateNumberPasLevels(maxPAS);
                 UserConfigTask_UpdatePasMaxPower(pasMaxPower);
                 UserConfigTask_UpdateTorqueMinimumThresholdStartup(torqueMinimumThresholdStartup);      
                 UserConfigTask_UpdateStartupOffsetMinimumThresholdSpeed(startupTorqueMinimumThresholdSpeed);
                 UserConfigTask_UpdateTorqueMinimumThreshold(torqueMinimumThreshold);
                 UserConfigTask_UpdateTorqueSensorMultiplier(torqueSensorMultiplier);
                 UserConfigTask_UpdateTorqueMaxSpeed(torqueMaxSpeed);
                 
                 for(uint8_t n = PAS_0;n <= PAS_9;n++)
                 {
                    UserConfigTask_UpdateCadenceLevelSpeed(n, cadenceLevelSpeed[n]);
                    UserConfigTask_UpdateTorqueLevelPower(n, torqueLevelPower[n]);   
                 }
                 
                 UserConfigTask_UpdateBikeMaxSpeed(maxSpeed);
                 UserConfigTask_UpdateWalkModeSpeed(walkModeSpeed);
                 
                 //write in the data flash and reset the system.
                 UserConfigTask_WriteUserConfigIntoDataFlash(&UserConfigHandle);   
             }
        }
         
    }

}

/************* TASKS ****************/

/**
  *  It initializes the vehicle control application. Needs to be called before using
  *	 vehicle control related modules.
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

    #if (!GNR_IOT) | SUPPORT_SLAVE_ON_IOT
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

    #if SCREEN_PROTOCOL == UART_APT
        LCD_APT_init(&LCD_APT_handle, &VCInterfaceHandle, &UART0Handle);
    #elif SCREEN_PROTOCOL == UART_KD718
        LCD_KD718_init(&LCD_KD718_handle, &VCInterfaceHandle, &UART0Handle);
    #elif SCREEN_PROTOCOL == UART_CLOUD_5S
        LCD_Cloud_5S_init(&LCD_Cloud_5S_handle, &VCInterfaceHandle, &UART0Handle);
    #elif SCREEN_PROTOCOL == UART_LOG_HS
        LogHS_Init(&LogHS_handle, &VCInterfaceHandle, &UART0Handle);
    #else
    #endif 
}

__NO_RETURN void ProcessUARTFrames (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	while(true)
	{
		osThreadFlagsWait(UART_FLAG, osFlagsWaitAny, osWaitForever);

        #if SCREEN_PROTOCOL == UART_APT
            LCD_APT_Task(&LCD_APT_handle); //Run the APT task
        #elif SCREEN_PROTOCOL == UART_KD718
            LCD_KD718_Task(&LCD_KD718_handle); //Run the KD718 task
        #elif SCREEN_PROTOCOL == UART_CLOUD_5S
            LCD_Cloud_5S_Task(&LCD_Cloud_5S_handle); //Run the Cloud 5S task     
        #elif SCREEN_PROTOCOL == UART_LOG_HS
            LogHS_ProcessFrame(&LogHS_handle);
        #else
        #endif 
	}
}


/**
  Function to configure and initialize the CANOPEN node.
*/
void CANOpenTask(void)
{

    //setup can OD and node ID.
    CO_SelecOdSetup(VcAutodeter_GetGnrState());

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
}

/**
  Function to decide if some IOT functions must be used or
  not to some bike models.
*/
bool Comm_CheckIotUsage(void)
{
    //if any of the vehicle tested below
    //return false to indicate that vehicle doesn't need
    //a specific functionality from IOT setup.
    if ((VEHICLE_SELECTION == VEHICLE_NIDEC))
    {
        return false;
    }
    else
    {
        return true;
    }   
}

/**
  * Initialises the object dictionairy with the user config values
  */
void Comm_InitODWithUserConfig(CO_NODE *pNode)
{
   static bool bHasInit = false;

   if(!bHasInit && CONmtGetMode(&pNode->Nmt) == CO_OPERATIONAL)
   {
        bHasInit = true;
       
        uint32_t hFwVersion;       
    
        //get dfu pack version
        hFwVersion = GnrInfo_GetDFuPackVersion();
    
        //get the serial number
        uint64_t fserialNumber = GnrInfo_GetSerialNumber();
    
        //transfer the serial number to the variables used by the CAN OD.
        uint32_t fSerialNbLow   = (uint32_t)(fserialNumber);
        uint32_t fSerialNbHigh   = (uint32_t)(fserialNumber >> 32);
    
     
        /***************Throttle/Pedal Assist variables******************************/
        uint8_t pasAlgorithm                       = UserConfigTask_GetPasAlgorithm();
        uint8_t maxPAS                             = UserConfigTask_GetNumberPasLevels();
        uint8_t pasMaxPower                        = UserConfigTask_GetPasMaxPower();
        uint8_t torqueMinimumThresholdStartup      = UserConfigTask_GetTorqueMinimumThresholdStartup();    
        uint8_t startupTorqueMinimumThresholdSpeed = UserConfigTask_GetStartupOffsetMinimumThresholdSpeed();
        uint8_t torqueMinimumThreshold             = UserConfigTask_GetTorqueMinimumThreshold();
        uint8_t torqueSensorMultiplier             = UserConfigTask_GetTorqueSensorMultiplier();
        uint8_t torqueMaxSpeed                     = UserConfigTask_GetTorqueMaxSpeed();
    
        uint8_t cadenceLevelSpeed[10] =      {UserConfigTask_GetCadenceLevelSpeed(PAS_0),UserConfigTask_GetCadenceLevelSpeed(PAS_1),
                                              UserConfigTask_GetCadenceLevelSpeed(PAS_2),UserConfigTask_GetCadenceLevelSpeed(PAS_3),
                                              UserConfigTask_GetCadenceLevelSpeed(PAS_4),UserConfigTask_GetCadenceLevelSpeed(PAS_5),
                                              UserConfigTask_GetCadenceLevelSpeed(PAS_6),UserConfigTask_GetCadenceLevelSpeed(PAS_7),
                                              UserConfigTask_GetCadenceLevelSpeed(PAS_8),UserConfigTask_GetCadenceLevelSpeed(PAS_9)};
        uint8_t torqueLevelPower[10] =       {UserConfigTask_GetTorqueLevelPower(PAS_0),UserConfigTask_GetTorqueLevelPower(PAS_1),
                                              UserConfigTask_GetTorqueLevelPower(PAS_2),UserConfigTask_GetTorqueLevelPower(PAS_3),
                                              UserConfigTask_GetTorqueLevelPower(PAS_4),UserConfigTask_GetTorqueLevelPower(PAS_5),
                                              UserConfigTask_GetTorqueLevelPower(PAS_6),UserConfigTask_GetTorqueLevelPower(PAS_7),
                                              UserConfigTask_GetTorqueLevelPower(PAS_8),UserConfigTask_GetTorqueLevelPower(PAS_9)};
        uint8_t maxSpeed = UserConfigTask_GetBikeMaxSpeed();
        uint8_t walkModeSpeed = UserConfigTask_GetWalkModeSpeed(); 
   
   
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SERIAL_NB, M2)),     pNode, &fSerialNbLow, sizeof(fSerialNbLow));     
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SERIAL_NB, M1)),     pNode, &fSerialNbHigh,  sizeof(fSerialNbHigh));  
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FW_VERSION, M1)),    pNode, &hFwVersion, sizeof(uint32_t));           
        
        /***********************UPdate Throttle/Pedal Assist CANopen OD ID.********************************************/
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_ALGORITHM, 0)),            pNode, &pasAlgorithm, sizeof(uint8_t));      
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_PAS, 0)),                  pNode, &maxPAS, sizeof(uint8_t));            
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MAX_POWER, 0)),            pNode, &pasMaxPower, sizeof(uint8_t));             
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 0)), pNode, &torqueMinimumThreshold, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 1)), pNode, &torqueMinimumThresholdStartup, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MINIMUM_THRESHOLD, 2)), pNode, &startupTorqueMinimumThresholdSpeed, sizeof(uint8_t));                                              
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, 0)), pNode, &torqueSensorMultiplier, sizeof(uint8_t));    
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_MAX_SPEED, 0)),         pNode, &torqueMaxSpeed, sizeof(uint8_t));            
        
        //fill the OD ID to cadenceLevelSpeed and torqueLevelPower with the current values.
        //this OD ID have 10 subindex each.
        for(uint8_t n = PAS_0;n <= PAS_9;n++)                                                                                                      
        {
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CADENCE_HYBRID_LEVEL, n)), pNode, &cadenceLevelSpeed[n], sizeof(uint8_t)); 
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_LEVEL_POWER, n)), pNode, &torqueLevelPower[n], sizeof(uint8_t));          
        }
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_SPEED, 0)), pNode, &maxSpeed, sizeof(uint8_t));                                 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 0)), pNode, &walkModeSpeed, sizeof(uint8_t));                       
   }           
}


#ifdef CANLOGGERTASK

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
            CANLog_SendVbus(&CoCanDriver,&VCInterfaceHandle); //Send Vbus
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
            CANLog_SendThrottleBrake(&CoCanDriver, &VCInterfaceHandle); //Send throttle & brake info*/
        }    
    }
}

#endif
