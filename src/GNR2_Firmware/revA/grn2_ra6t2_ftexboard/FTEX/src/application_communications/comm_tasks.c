/**
  * @file    comm_tasks.c
  * @brief   This module gather
  *
  */
#include "comm_tasks.h"
#include "vc_tasks.h"

#include "comm_config.h"
#include "vc_config.h"
#include "Utilities.h"
#include "gnr_main.h"
#include "vc_autodetermination.h"
#include "board_hardware.h"
#include "uCAL_GPIO.h"

#include "vc_errors_management.h"

// CANOpen includes
#include "co_core.h"
#include "co_gnr2_specs.h"
// CAN logger
#include "can_logger.h"
#include "can_vehicle_interface.h"

#include "lcd_apt.h"
#include "wheel.h"
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

/** @brief  Callback function used for updating the values of the GNR
*           object dictionary.
*/
static void UpdateObjectDictionnary(void *p_arg)
{

    CO_NODE  *pNode;
    pNode = (CO_NODE *)p_arg;
    
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    // Get Bike Parameters 
    // Read only
    uint8_t     hSpeed;
    uint16_t    hSpeedDec;
    uint16_t    hDCPWR;    
    uint16_t    hTorque;
    uint16_t    hPWR;
    uint16_t    hMaxDCPwr;
     int16_t    hMotorTemp;
     int16_t    hHeatsinkTemp;
     uint8_t    bSOC;
    uint8_t     hBrakeStatus;
    uint32_t    hErrorState;
    uint16_t    hBusVoltage;
    int16_t     configMotorRpm;
    uint32_t    odometerDistance;
    int16_t     configMotorRpmWithGearRatio;
    int16_t     hPhaseCurrentSensor1;
    int16_t     hPhaseCurrentSensor2;
    uint16_t    PedalRPM;
    uint8_t     PerdalTorqPercent;
    uint8_t     PowertrainLockUnlock;
    // Read and write
    uint8_t     bPAS[2];
    uint8_t     hWheelDiameter[2];
    uint8_t     hFrontLightState[2];
    uint8_t     hRearLightState[2];
    //canopen algorithm selection
    static uint8_t CanOpenSetAlgorithm = 0;
    
    
    ASSERT(pVCI != NULL);
    ASSERT(pVCI->pPowertrain != NULL);
    ASSERT(pVCI->pPowertrain->pPAS != NULL);
    
    //only master can access theses parameters.
    if (VcAutodeter_GetGnrState())
    {
        hSpeed               = (uint8_t) CanVehiInterface_GetVehicleSpeed();
        hSpeedDec            = Wheel_GetVehicleHectometerSpeedFromWSS();
        hDCPWR               = CanVehiInterface_GetVehicleDCPower(pVCI);
        hTorque              = CanVehiInterface_GetVehicleTorque(pVCI);
        hPWR                 = CanVehiInterface_GetVehiclePower(pVCI);
        hMaxDCPwr            = CanVehiInterface_GetMaxDCPWR(pVCI);
        hMotorTemp           = CanVehiInterface_GetMotorTemp(pVCI);
        hHeatsinkTemp        = CanVehiInterface_GetControllerTemp(pVCI);
        bSOC                 = CanVehiInterface_GetVehicleSOC(pVCI);
        hErrorState          = CanVehiInterface_GetVehicleCurrentFaults(pVCI);
        hBusVoltage          = CanVehiInterface_GetBusVoltage();
        hBrakeStatus         = CanVehiInterface_GetBrakeStatus(pVCI);
        hPhaseCurrentSensor1 = CanVehiculeInterface_GetSensorPhaseCurrentRMS(CURRENT_SENSOR_1);
        hPhaseCurrentSensor2 = CanVehiculeInterface_GetSensorPhaseCurrentRMS(CURRENT_SENSOR_2);
			
		odometerDistance	 = CanVehiInterface_GetOdometerDistance();
        
        configMotorRpm = UserConfigTask_GetMotorRpm(pVCI);
        configMotorRpmWithGearRatio = UserConfigTask_GetMotorRpmWithGearRatio(pVCI);
        
        bPAS[VEHICLE_PARAM]             = CanVehiInterface_GetVehiclePAS(pVCI); 
        hFrontLightState[VEHICLE_PARAM] = CanVehiInterface_GetFrontLightState(pVCI);
        hRearLightState[VEHICLE_PARAM]  = CanVehiInterface_GetRearLightState(pVCI);
        hWheelDiameter[VEHICLE_PARAM]   = CanVehiInterface_GetWheelDiameter();
        PedalRPM                        = CanVehiInterface_GetVehiclePedalRPM();
        PerdalTorqPercent               = CanVehiInterface_GetPedalTorqPercentage(pVCI);
    }
    else
    {
        hErrorState = 0x0000;
    }
    
    /***************Throttle/Pedal Assist variables******************************/
    uint8_t maxPAS;
    uint8_t pasMaxTorqueRatio;
    uint16_t torqueSensorMultiplier[9];
    uint8_t pasLevelMinTorque[9];

    uint16_t pasStartupSpeedThreshold;                 
    uint16_t pasRuntimeSpeedThreshold;                
    uint16_t pasTorqueDetectionValue;                 
    uint16_t pasStartupPulses;                 
    uint16_t pasStartupTimeWindow;               
    uint16_t pasRuntimePulses;                        
    uint16_t pasRuntimeTimeWindow;                
    uint8_t  pasCadenceAndOrTorque;                    
    uint8_t  pasTorqueScalingPedalRPM;                 
    uint16_t pasTorqueScalingMinRPM;                   
    uint16_t pasTorqueScalingMaxRPM;                  
    uint16_t pasTorqueScalingMinRPMGain;               
    uint16_t pasTorqueScalingMaxRPMGain;               
    uint8_t  pasRampType;                              
    uint16_t dynamicDecelerationRampStart;             
    uint16_t dynamicDecelerationRampEnd;               
    uint16_t dynamicDecelerationRampMinDeceleration;   
    uint16_t dynamicDecelerationRampMaxDeceleration;   
    uint16_t highSpeedPowerLimitingRampStart;          
    uint16_t highSpeedPowerLimitingRampEnd;            
    uint16_t highSpeedPowerLimitingRampMinSpeedPower;  
    uint16_t highSpeedPowerLimitingRampMaxSpeedPower;  
    
    uint8_t PasLevelSpeed[9];
    uint8_t pasLevelMaxTorque[9];
    uint8_t maxSpeed;
    uint8_t walkModeSpeed;
                                              
    /***********variable used to get the key code that enable user data configuration to be updated.**************/
    uint16_t keyUserDataConfig = 0;
                                          
    //variable used to verify if a firmware update command was received or not.
    uint8_t FirmwareUpdateCommand = 0;                                           
    
    //
    #if SUPPORT_SLAVE_ON_IOT | !GNR_IOT
    /* Get data from motor control and vehicle control layer */
    int16_t hMotorSpeedMeas         = MDI_GetAvrgMecSpeedUnit(pVCI->pPowertrain->pMDI, M1);
    uint16_t hMotorState            = MDI_GetSTMState(pVCI->pPowertrain->pMDI, M1);
    uint32_t wMotorOccurredFaults    = MDI_GetOccurredCriticalFaults(pVCI->pPowertrain->pMDI, M1);
    uint32_t wMotorCurrentFaults    = MDI_GetCurrentCriticalFaults(pVCI->pPowertrain->pMDI, M1);
    uint32_t wMotorCurrentErrorsNow        = MDI_GetCurrentErrors(pVCI->pPowertrain->pMDI, M1);
    uint32_t wMotorOccurredErrors    = MDI_GetOccurredErrors(pVCI->pPowertrain->pMDI, M1);
    uint32_t wMotorWarnings         = MDI_GetOccurredWarnings(pVCI->pPowertrain->pMDI, M1);
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
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M1)), pNode, &hMotorState, sizeof(hMotorState));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 0)), pNode, &wMotorWarnings, sizeof(wMotorWarnings));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 2)), pNode, &wMotorCurrentErrorsNow, sizeof(wMotorCurrentErrorsNow));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 4)), pNode, &wMotorOccurredErrors, sizeof(wMotorOccurredErrors));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 6)), pNode, &wMotorCurrentFaults, sizeof(wMotorCurrentFaults));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 8)), pNode, &wMotorOccurredFaults, sizeof(wMotorOccurredFaults));
            
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
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M1)), pNode, &hMotorState, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 0)), pNode, &wMotorWarnings, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 2)), pNode, &wMotorCurrentErrorsNow, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 4)), pNode, &wMotorOccurredErrors, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 6)), pNode, &wMotorCurrentFaults, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 8)), pNode, &wMotorOccurredFaults, sizeof(uint32_t));
                
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
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M2)), pNode, &hHeatsinkTemp, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_STATE, M2)), pNode, &hMotorState, sizeof(uint16_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 1)), pNode, &wMotorWarnings, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 3)), pNode, &wMotorCurrentErrorsNow, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 5)), pNode, &wMotorOccurredErrors, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 7)), pNode, &wMotorCurrentFaults, sizeof(uint32_t));
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_FAULTS, 9)), pNode, &wMotorOccurredFaults, sizeof(uint32_t));
                
                /* Read commands in CANOpen object dictionnary received by RPDO */
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TORQUE_REF, M2)), pNode, &hMotor2TorqRef, sizeof(uint16_t));
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_START, M2)), pNode, &bMotor2Start, sizeof(uint8_t));
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, sizeof(uint8_t));
                
                /* Execute received commands using motor control api */
                MDI_ExecTorqueRamp(pVCI->pPowertrain->pMDI, M1, hMotor2TorqRef);
                bMotor2Start ? MDI_StartMotor(pVCI->pPowertrain->pMDI, M1) : MDI_StopMotor(pVCI->pPowertrain->pMDI, M1);
                if (bMotor2FaultAck)
                {
                    // Reset fault ack after reception
                    bMotor2FaultAck = 0;
                    //critical section
                    COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FAULT_ACK, M2)), pNode, &bMotor2FaultAck, sizeof(uint8_t));
                    //end critical section
                    MDI_CriticalFaultAcknowledged(pVCI->pPowertrain->pMDI, M1);
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
                    MDI_StopMotor(pVCI->pPowertrain->pMDI, M1);
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
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)),     pNode, &bPAS[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 0)),     pNode, &hWheelDiameter[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0)), pNode, &hFrontLightState[CAN_PARAM], sizeof(uint8_t));
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 0)),  pNode, &hRearLightState[CAN_PARAM], sizeof(uint8_t));

            // Set the lock/unlock status over CAN
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_LOCK_UNLOCK_POWERTRAIN, 0)),  pNode, &PowertrainLockUnlock, sizeof(uint8_t));
            CanVehiInterface_SetPowertrainLockStatus(pVCI, PowertrainLockUnlock);
            
            //used to get the current algorithm in the OD.
            uint8_t CanOpenAlgorithm = 0;
            
            //read and check if PAS algorithm was changed by CANOPEN interface.
            COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CAN_SCREEN, 0)),  pNode, &CanOpenAlgorithm, sizeof(uint8_t));
            
            //verify if PAS algorithm must to be changed.
            if (((CanOpenAlgorithm  == TorqueSensorUse) || ((CanOpenAlgorithm  == CadenceSensorUse))) && (CanOpenAlgorithm != CanOpenSetAlgorithm) )
            {
                CanVehiInterface_SetAlgorithm(pVCI, CanOpenAlgorithm);
            }
            
            //hold the last received value.
            CanOpenSetAlgorithm = CanOpenAlgorithm;
             
            // Check if there were changes made to a parameter by a can device
            
            bool WriteOBJDict = false;
            
            if(UART0Handle.UARTProtocol == UART_APT)
            {   
                if(bPAS[VEHICLE_PARAM] != bPAS[CAN_PARAM])
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
            }
            else if(UART0Handle.UARTProtocol == UART_CLOUD_5S)
            {
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
            }
            else if (CanVehiInterface_CheckCANScreenSetup()) // If we have a can screen just reflect the change
            {
                CanVehiInterface_SetVehiclePAS (&VCInterfaceHandle, bPAS[CAN_PARAM]);  // propagate the change in the vehicle 
            }
            
            if(CanVehiInterface_CheckCANScreenSetup()) // If we are allowed to have a CAN screen
            {
                uint16_t ExternalThrottleVal = 0;
                uint8_t  ExternalCruiseControlState = 0;
                static uint8_t LastCANCruiseState = 0;
                
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 1)),  pNode, &ExternalThrottleVal, sizeof(uint16_t));
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_CRUISE, 0)),  pNode, &ExternalCruiseControlState, sizeof(uint8_t));
                
                if(CO_CheckCANHB(&CanScreenHB) == true)// Is the screen heart beat active ?
                {                                       
                    // Keep the throttle value up to date
                    CanVehiInterface_UpdateExternalThrottle(&VCInterfaceHandle,ExternalThrottleVal);
                    
                    if (LastCANCruiseState == 1 && CanVehiInterface_GetCruiseControlState(&VCInterfaceHandle) == false) // Did the controller force a disengage of cruise control
                    {                                               
                        if (ExternalCruiseControlState == 1) // Does CAN still think cruise is On ?
                        { 
                           // Force the change on CAN
                           ExternalCruiseControlState = 0;
                           COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_CRUISE, 0)),  pNode, &ExternalCruiseControlState, sizeof(uint8_t)); 
                        }
                        else
                        {
                           LastCANCruiseState = 0;
                        }                        
                    }
                    else if (ExternalCruiseControlState == 1 && LastCANCruiseState == 0)
                    {
                        CanVehiInterface_EngageCruiseControl(&VCInterfaceHandle);
                        LastCANCruiseState = 1;
                    }
                    else if (ExternalCruiseControlState == 0 && LastCANCruiseState == 1)
                    { 
                        CanVehiInterface_DisengageCruiseControl(&VCInterfaceHandle);
                        LastCANCruiseState = 0;
                    }                          
                }  
                // If we haven't received anything from the screen and the heartbeat overflowed
                else if (ExternalThrottleVal > 0 || ExternalCruiseControlState == 1)
                {                    
                    CanVehiInterface_UpdateExternalThrottle(&VCInterfaceHandle,0);
                    CanVehiInterface_DisengageCruiseControl(&VCInterfaceHandle);
                        
                    ExternalThrottleVal = 0;    // Reflect the change on CAN
                    COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 1)),  pNode, &ExternalThrottleVal, sizeof(uint16_t));
                    ExternalCruiseControlState = 0;
                    COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_CRUISE, 0)),  pNode, &ExternalCruiseControlState, sizeof(uint8_t));                    
                }
            }   
            
            if(hWheelDiameter[VEHICLE_PARAM] != hWheelDiameter[CAN_PARAM])
            {
                if (Wheel_CheckInternalUpdateFlag()) // Check if the change came from the vehicle
                {
                     hWheelDiameter[CAN_PARAM] = hWheelDiameter[VEHICLE_PARAM];
                }   
                else // If not it's a CAN change
                {                   
                     CanVehiInterface_UpdateWheelDiameter(hWheelDiameter[CAN_PARAM]);
                }            
            }
           
            // If the light status in OBJ dict and vehicle don't match, update the one in the vehicle
            if(hFrontLightState[VEHICLE_PARAM] != hFrontLightState[CAN_PARAM])
            {
                if(Light_CheckInternalUpdateFlag(VCInterfaceHandle.pPowertrain->pHeadLight))
                {
                    hFrontLightState[CAN_PARAM] = hFrontLightState[VEHICLE_PARAM];
                    Light_ClearInternalUpdateFlag(VCInterfaceHandle.pPowertrain->pHeadLight);
                }
                else
                {
                    CanVehiInterface_ChangeFrontLightState(&VCInterfaceHandle,hFrontLightState[CAN_PARAM]);
                }                           
            }
            
            if(hRearLightState[VEHICLE_PARAM] != hRearLightState[CAN_PARAM])
            {
                if(Light_CheckInternalUpdateFlag(VCInterfaceHandle.pPowertrain->pTailLight))
                {
                    hRearLightState[CAN_PARAM] = hRearLightState[VEHICLE_PARAM];
                    Light_ClearInternalUpdateFlag(VCInterfaceHandle.pPowertrain->pTailLight);
                }
                else
                {
                    CanVehiInterface_ChangeRearLightState(&VCInterfaceHandle,hRearLightState[CAN_PARAM]);                   
                }                    
            }
            
                                  
            /**************Write the repesctive OD ID, updating the OD that us read by the IOT module using SDO.*************/
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_MEASURE, 0)), pNode, &hSpeed, sizeof(uint8_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_MEASURE, 1)), pNode, &hSpeedDec, sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_POWER_MEASURE, 0 )), pNode, &hDCPWR, sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_POWER_MEASURE, 1 )), pNode, &hTorque, sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_POWER_MEASURE, 2 )), pNode, &hPWR, sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMP,    M1)), pNode, &hMotorTemp, sizeof(int16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_HEATSINK_TEMP, M1)), pNode, &hHeatsinkTemp, sizeof(int16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SOC, M1)),           pNode, &bSOC, sizeof(uint8_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_ERR_STATE, M1)),     pNode, &hErrorState, sizeof(uint32_t));

            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_DCPOWER, M1)),   pNode, &hMaxDCPwr, sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BUS_VOLTAGE, M1)),   pNode, &hBusVoltage, sizeof(uint16_t));
            
            if (WriteOBJDict)
            {    
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL, M1)),     pNode, &bPAS[CAN_PARAM], sizeof(uint8_t));
            }
            

            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 0)),pNode, &hWheelDiameter[CAN_PARAM], sizeof(uint8_t));
            
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0)), pNode, &hFrontLightState[CAN_PARAM], sizeof(uint8_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 0)),  pNode, &hRearLightState[CAN_PARAM], sizeof(uint8_t));

            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BRAKE, 0)), pNode, &hBrakeStatus, sizeof(uint8_t));
            
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_RPM, 0)),      pNode, &configMotorRpm, sizeof(int16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_RPM, 1)),      pNode, &configMotorRpmWithGearRatio, sizeof(int16_t));
						
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_ODOMETER_DISTANCE, 0)),      pNode, &odometerDistance, sizeof(uint32_t));
						

            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SENSOR_CURRENT, 0)), pNode, &hPhaseCurrentSensor1, sizeof(int16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_SENSOR_CURRENT, 1)), pNode, &hPhaseCurrentSensor2, sizeof(int16_t));          
            
            
            //PAS sensor information
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 0 )), pNode, &PedalRPM, sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 1 )), pNode, &PerdalTorqPercent, sizeof(uint8_t));
            
            //Update pasLevelMinTorque
            CanVehiInterface_GetPasLevelMinTorque(&VCInterfaceHandle, pasLevelMinTorque);

            //Update the minimum torque percentage value to all PAS level.
            for(uint8_t n = PAS_LEVEL_0;n <= PAS_LEVEL_9;n++)
            {
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MIN_TORQUE, n)), pNode, &pasLevelMinTorque[n], sizeof(uint8_t));
            }
            
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
            uint8_t  configPasNbMagnetsPerTurn = UserConfigTask_GetPasNbMagnetsPerTurn();                                     
            uint16_t configPasTorqueInputMax = UserConfigTask_GetPasTorqueInputMax();
            uint16_t configPasTorqueInputMin = UserConfigTask_GetPasTorqueInputMin();
            
            uint8_t configWheelSpeedSensorNbrMagnets;
            uint8_t configWheelDiameter;
            uint8_t configScreenProtocol;
            
            bool configisMotorMixedSignal;
            uint16_t configMinSignalThreshold;
            uint32_t configMaxWheelSpeedPeriodUs;
    
            uint8_t configHeadLightDefault;
            uint8_t configTailLightDefault;
            uint8_t configTailLightBlinkOnBrake;

            uint16_t configThrottleAdcOffset;
            uint16_t configThrottleAdcMax; 
            
            uint8_t configPasOverThrottle;
            
            uint8_t  configWalkmodeMaxTorque;
            uint8_t  configWalkmodeAccelRampType;
            uint16_t configWalkmodeAccelRampArg1;
            
            uint16_t pasLowPassFilterBW1[BW_ARRAY_SIZE]; 
            uint16_t pasLowPassFilterBW2[BW_ARRAY_SIZE]; 
            uint8_t  FilterSpeed[FILTERSPEED_ARRAY_SIZE];
            
            uint16_t configBatteryFullVoltage;       
            uint16_t configBatteryEmptyVoltage;          
            uint16_t configBatteryMaxPeakDCCurrent;
            uint16_t configBatteryContinuousDCCurrent;        
            uint16_t configBatteryPeakCurrentMaxDuration;
            uint16_t configBatteryPeakCurrentDeratingDuration;
            
            uint8_t  configThrottleBlockOff;
            uint8_t  configThrottleMaxSpeed;
            uint8_t  configThrottleAccelRampType;
            uint16_t configThrottleAccelRampArg1;
            
            uint8_t motorTempSensorType;
            uint16_t motorNTCBetaCoef;
            uint16_t motorNTCResistanceCoef;
            
            //CAN terminator resistor default state
            uint8_t configCANresistorStatus;
            
            //verify is user data config is ready to be write in data flash memory.
             if(keyUserDataConfig == KEY_USER_DATA_CONFIG_UPDATED)
             {
                 /**********read value hold by the OD****************/
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_PAS,               0)), pNode, &maxPAS, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MAX_TORQUE_RATIO,  0)), pNode, &pasMaxTorqueRatio, sizeof(uint8_t));
        
                 for(uint8_t n = 0; n < 9; n++)
                 {
                    COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, n)), pNode, &torqueSensorMultiplier[n], sizeof(uint16_t));
                    COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL_SPEED, n)), pNode, &PasLevelSpeed[n], sizeof(uint8_t));
                    COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MIN_TORQUE, n)), pNode, &pasLevelMinTorque[n], sizeof(uint8_t));
                    COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MAX_TORQUE, n)), pNode, &pasLevelMaxTorque[n], sizeof(uint8_t));                                                               
                 }
                
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 8)), pNode, &configPasOverThrottle, sizeof(uint8_t));
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_SPEED, 0)), pNode, &maxSpeed, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 0)), pNode, &walkModeSpeed, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 1)), pNode, &configWalkmodeMaxTorque, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 2)), pNode, &configWalkmodeAccelRampType, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 3)), pNode, &configWalkmodeAccelRampArg1, sizeof(uint16_t));
                
                 // Config 
                 // Subindex 0-2 are read only and not in the memory section, placeholder for index 2 is not implemented yet.
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 3)),       pNode, &configPasNbMagnetsPerTurn, sizeof(uint8_t));  
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 4)),       pNode, &configPasTorqueInputMin, sizeof(uint16_t));  
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 5)),       pNode, &configPasTorqueInputMax, sizeof(uint16_t)); 
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_VOLTAGE, 0)),        pNode, &configBatteryFullVoltage, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_VOLTAGE, 1)),        pNode, &configBatteryEmptyVoltage, sizeof(uint16_t));
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 1)),      pNode, &configWheelSpeedSensorNbrMagnets, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 2)),      pNode, &configWheelDiameter, sizeof(uint8_t)); 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_SCREEN_PROTOCOL, 0)),   pNode, &configScreenProtocol, sizeof(uint8_t));

                 //Motor signals parameters  
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 4)), pNode, &configisMotorMixedSignal, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 5)), pNode, &configMinSignalThreshold, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 6)), pNode, &configMaxWheelSpeedPeriodUs, sizeof(uint32_t));
                 
                 // Headlight default state
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 1)),  pNode, &configHeadLightDefault, sizeof(uint8_t));
                 // Taillight default state and blink on brake state
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 1)),   pNode, &configTailLightDefault, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 2)),   pNode, &configTailLightBlinkOnBrake, sizeof(uint8_t));

                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 0)),     pNode, &configBatteryMaxPeakDCCurrent, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 1)),     pNode, &configBatteryContinuousDCCurrent, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 2)),     pNode, &configBatteryPeakCurrentMaxDuration, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 3)),     pNode, &configBatteryPeakCurrentDeratingDuration, sizeof(uint16_t));               
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 2)),         pNode, &configThrottleAdcOffset, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 3)),         pNode, &configThrottleAdcMax, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 4)),         pNode, &configThrottleBlockOff, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 5)),         pNode, &configThrottleMaxSpeed, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 6)),         pNode, &configThrottleAccelRampType, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 7)),         pNode, &configThrottleAccelRampArg1, sizeof(uint16_t));
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER, 0)),      pNode, &FilterSpeed[0], sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER, 1)),      pNode, &FilterSpeed[1], sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 0)),      pNode, &pasLowPassFilterBW1[0], sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 1)),      pNode, &pasLowPassFilterBW2[0], sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 2)),      pNode, &pasLowPassFilterBW1[1], sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 3)),      pNode, &pasLowPassFilterBW2[1], sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 4)),      pNode, &pasLowPassFilterBW1[2], sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 5)),      pNode, &pasLowPassFilterBW2[2], sizeof(uint16_t));    

                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMPERATURE, 0)),    pNode, &motorTempSensorType, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMPERATURE, 1)),    pNode, &motorNTCBetaCoef, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMPERATURE, 2)),    pNode, &motorNTCResistanceCoef, sizeof(uint16_t));

                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_THRESHOLD, 0)),        pNode, &pasStartupSpeedThreshold, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_THRESHOLD, 1)),        pNode, &pasRuntimeSpeedThreshold, sizeof(uint16_t));
                                          
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 0)),   pNode, &pasTorqueDetectionValue, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 1)),   pNode, &pasStartupPulses, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 2)),   pNode, &pasStartupTimeWindow, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 3)),   pNode, &pasRuntimePulses, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 4)),   pNode, &pasRuntimeTimeWindow, sizeof(uint16_t));
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CADENCE_AND_OR_TORQUE, 0)),      pNode, &pasCadenceAndOrTorque, sizeof(uint8_t));
         
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 0)),   pNode, &pasTorqueScalingPedalRPM, sizeof(uint8_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 1)),   pNode, &pasTorqueScalingMinRPM, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 2)),   pNode, &pasTorqueScalingMaxRPM, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 3)),   pNode, &pasTorqueScalingMinRPMGain, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 4)),   pNode, &pasTorqueScalingMaxRPMGain, sizeof(uint16_t));
         
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_RAMP_SELECTION, 0)),         pNode, &pasRampType, sizeof(uint8_t));
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 0)), pNode, &dynamicDecelerationRampStart, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 1)), pNode, &dynamicDecelerationRampEnd, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 2)), pNode, &dynamicDecelerationRampMaxDeceleration, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 3)), pNode, &dynamicDecelerationRampMinDeceleration, sizeof(uint16_t));
                 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 0)), pNode, &highSpeedPowerLimitingRampStart, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 1)), pNode, &highSpeedPowerLimitingRampEnd, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 2)), pNode, &highSpeedPowerLimitingRampMinSpeedPower, sizeof(uint16_t));
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 3)), pNode, &highSpeedPowerLimitingRampMaxSpeedPower, sizeof(uint16_t));
                 
                 
                 //Read the CAN Resistor status to update 
                 COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CAN_TERMINAL_RESISTOR, 0)),    pNode, &configCANresistorStatus, sizeof(uint8_t));
                 
                 /******update all variables used to keep the user data config that will be written in to the usaer data flash.****/
                 
                 //update Throttle/Pedal Assist variables that will be write into the user data flash.
                 
                 UserConfigTask_UpdateMotorSensorType(motorTempSensorType);
                 UserConfigTask_UpdateMotorNTCBetaCoef(motorNTCBetaCoef);
                 UserConfigTask_UpdateMotorNTCResistanceCoef(motorNTCResistanceCoef);
                 
                 UserConfigTask_UpdateNumberPasLevels(maxPAS);
                 UserConfigTask_UpdatePasMaxTorqueRatio(pasMaxTorqueRatio);
                 
                 for(uint8_t n = PAS_1;n <= PAS_9;n++)
                 {
                    UserConfigTask_UpdateTorqueSensorMultiplier(n, torqueSensorMultiplier[n-1]);
                    UserConfigTask_UpdatePasLevelSpeed(n, PasLevelSpeed[n-1]);
                    UserConfigTask_UpdatePasLevelMinTorque(n, pasLevelMinTorque[n-1]);
                    UserConfigTask_UpdatePasLevelMaxTorque(n, pasLevelMaxTorque[n-1]);                      
                 }
                 
                 UserConfigTask_UpdatePASOverThrottle(configPasOverThrottle);
                 
                 UserConfigTask_UpdateBikeMaxSpeed(maxSpeed);
                 UserConfigTask_UpdateWalkmodeSpeed(walkModeSpeed);
                 UserConfigTask_UpdateWalkmodeMaxTorque(configWalkmodeMaxTorque);
                 UserConfigTask_UpdateWalkmodeAccelRampType(configWalkmodeAccelRampType);
                 UserConfigTask_UpdateWalkmodeAccelRampArg1(configWalkmodeAccelRampArg1);
                     
                 UserConfigTask_UpdatePasNbMagnetsPerTurn(configPasNbMagnetsPerTurn);                                     
                 UserConfigTask_UpdatePasTorqueInputMax(configPasTorqueInputMax);
                 UserConfigTask_UpdatePasTorqueInputMin(configPasTorqueInputMin);
                                  
                 UserConfigTask_UpdateWheelSpeedSensorNbrMagnets(configWheelSpeedSensorNbrMagnets);
                 UserConfigTask_UpdateWheelDiameter(configWheelDiameter);
                 UserConfigTask_UpdateScreenProtocol(configScreenProtocol);
                 
                 UserConfigTask_UpdateMotorMixedSignalState(configisMotorMixedSignal);
                 UserConfigTask_UpdateMinSignalThreshold(configMinSignalThreshold);
                 UserConfigTask_UpdateMaxWheelSpeedPeriodUs(configMaxWheelSpeedPeriodUs);
                                 
                 UserConfigTask_UpdateHeadLightDefault(configHeadLightDefault);
                 
                 UserConfigTask_UpdateTailLightDefault(configTailLightDefault);
                 UserConfigTask_UpdateTailLightBlinkOnBrake(configTailLightBlinkOnBrake);                 
                 
                 UserConfigTask_UpdateThrottleAdcOffset(configThrottleAdcOffset);
                 UserConfigTask_UpdateThrottleAdcMax(configThrottleAdcMax);
                 UserConfigTask_UpdateThrottleBlockOff(configThrottleBlockOff);
                 UserConfigTask_UpdateThrottleMaxSpeed(configThrottleMaxSpeed);
                 UserConfigTask_UpdateThrottleAccelRampType(configThrottleAccelRampType);
                 UserConfigTask_UpdateThrottleAccelRampArg1(configThrottleAccelRampArg1);
                 
                 
                 UserConfigTask_UpdateBatteryFullVoltage(configBatteryFullVoltage);
                 UserConfigTask_UpdateBatteryEmptyVoltage(configBatteryEmptyVoltage);
                 UserConfigTask_UpdateBatteryMaxPeakDCCurrent(configBatteryMaxPeakDCCurrent);
                 UserConfigTask_UpdateBatteryContinuousDCCurrent(configBatteryContinuousDCCurrent);        
                 UserConfigTask_UpdateBatteryPeakCurrentMaxDuration(configBatteryPeakCurrentMaxDuration);
                 UserConfigTask_UpdateBatteryPeakCurrentDeratingDuration(configBatteryPeakCurrentDeratingDuration);

                 UserConfigTask_SetPASStartupSpeedThreshold(pasStartupSpeedThreshold);
                 UserConfigTask_SetPASRuntimeSpeedThreshold(pasRuntimeSpeedThreshold);
                 UserConfigTask_SetPASTorqueDetectionValue(pasTorqueDetectionValue);
                 UserConfigTask_SetPASStartupPulses(pasStartupPulses);
                 UserConfigTask_SetPASStartupTimeWindow(pasStartupTimeWindow);
                 UserConfigTask_SetPASRuntimePulses(pasRuntimePulses);
                 UserConfigTask_SetPASRuntimeTimeWindow(pasRuntimeTimeWindow);
                 UserConfigTask_SetPASCadenceAndOrTorque(pasCadenceAndOrTorque);
                 UserConfigTask_SetPASTorqueScalingPedalRPMActivated(pasTorqueScalingPedalRPM);
                 UserConfigTask_SetPASTorqueScalingMinRPM(pasTorqueScalingMinRPM);
                 UserConfigTask_SetPASTorqueScalingMaxRPM(pasTorqueScalingMaxRPM);
                 UserConfigTask_SetPASTorqueScalingMinRPMGain(pasTorqueScalingMinRPMGain);
                 UserConfigTask_SetPASTorqueScalingMaxRPMGain(pasTorqueScalingMaxRPMGain);
                 UserConfigTask_SetPASRampType(pasRampType);
                 UserConfigTask_SetDynamicDecelerationRampStart(dynamicDecelerationRampStart);
                 UserConfigTask_SetDynamicDecelerationRampEnd(dynamicDecelerationRampEnd);
                 UserConfigTask_SetDynamicDecelerationMaxDeceleration(dynamicDecelerationRampMaxDeceleration);
                 UserConfigTask_SetDynamicDecelerationMinDeceleration(dynamicDecelerationRampMinDeceleration);
                 UserConfigTask_SetHighSpeedPowerLimitingRampStart(highSpeedPowerLimitingRampStart);
                 UserConfigTask_SetHighSpeedPowerLimitingRampEnd(highSpeedPowerLimitingRampEnd);
                 UserConfigTask_SetHighSpeedPowerLimitingRampMinSpeedPower(highSpeedPowerLimitingRampMinSpeedPower);
                 UserConfigTask_SetHighSpeedPowerLimitingRampMaxSpeedPower(highSpeedPowerLimitingRampMaxSpeedPower);  
                 
                 
                 //update speed values to bw filter.
                 for(uint8_t n = 0;n < FILTERSPEED_ARRAY_SIZE;n++)
                 {
                    UserConfigTask_UpdateFilterSpeed(n, FilterSpeed[n]);
                 }
                 
                 //update bw filter values to the speed intervals.
                 for(uint8_t n = 0;n < BW_ARRAY_SIZE;n++)
                 {
                    UserConfigTask_UpdateFilterBwValue(n, BW1, pasLowPassFilterBW1[n]);
                    UserConfigTask_UpdateFilterBwValue(n, BW2, pasLowPassFilterBW2[n]);
                 }
                 
                 UserConfigTask_UpdateCANterminatorState(configCANresistorStatus);
                 
                 //write in the data flash and reset the system.
                 UserConfigTask_WriteUserConfigIntoDataFlash(&UserConfigHandle); 
                                             
             }
        }      
    }
}

/************* TASKS ****************/

/**
  *  It initializes the vehicle control application. Needs to be called before using
  *     vehicle control related modules.
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
    
    //CAN Resistor termination configuration
    PinConfig.PinDirection = OUTPUT;
    PinConfig.PinPull      = NONE;
    PinConfig.PinOutput    = OPEN_DRAIN;
    uCAL_GPIO_ReInit(CAN_RES_ENABLE_GPIO_PIN, PinConfig);
    uCAL_GPIO_Set(CAN_RES_ENABLE_GPIO_PIN);
    if(0 == UserConfigTask_GetCANterminatorState()) 
    {
        uCAL_GPIO_Reset(CAN_RES_ENABLE_GPIO_PIN);
    }

    #if (!GNR_IOT) | SUPPORT_SLAVE_ON_IOT
    /* Initialize motor 2 handle to be used by vehicle control layer */
    SlaveMotorRegisterAddr_t M2RegAddr =
    {
        .wRegAddrMotorSpeed = CO_DEV(CO_OD_REG_MOTOR_SPEED, M2),
        .wRegAddrBusVoltage = CO_DEV(CO_OD_REG_BUS_VOLTAGE, M2),
        .wRegAddrWarnings = CO_DEV(CO_OD_REG_MOTOR_FAULTS, 1),
        .wRegAddrCurrentErrorsNow = CO_DEV(CO_OD_REG_MOTOR_FAULTS, 3),
        .wRegAddrOccurredErrors = CO_DEV(CO_OD_REG_MOTOR_FAULTS, 5),
        .wRegAddrCurrentFaults = CO_DEV(CO_OD_REG_MOTOR_FAULTS, 7),
        .wRegAddrOccurredFaults = CO_DEV(CO_OD_REG_MOTOR_FAULTS, 9),
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
            break;
        case UART_APT:
            LCD_APT_init(&LCD_APT_handle, &VCInterfaceHandle, &UART0Handle);
            break;
        case UART_KD718:
            LCD_KD718_init(&LCD_KD718_handle, &VCInterfaceHandle, &UART0Handle);
            break;
        case UART_CLOUD_5S:
            LCD_Cloud_5S_init(&LCD_Cloud_5S_handle, &VCInterfaceHandle, &UART0Handle);
            break; 
        case UART_LOG_HS:
            LogHS_Init(&LogHS_handle, &VCInterfaceHandle, &UART0Handle);
            break;
        case UART_DISABLE:
            // If we don;t use the UART assume we have a CAN screen
            CanVehiInterface_SetupCANScreen(&VCInterfaceHandle);
            CO_SetupCANHB(&CanScreenHB);
            break;
        default:
            //Dont initialise the euart
            break;
      }
}

__NO_RETURN void ProcessUARTFrames (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    VCI_Handle_t * pVCI = &VCInterfaceHandle;
    
    while(true)
    { 
        osThreadFlagsWait(UART_FLAG, osFlagsWaitAny, 500); // 500 ticks should correpsond to 250 ms
        
        if(uCAL_UART_GetTaskFlag(&UART0Handle))
        {            
            switch(UART0Handle.UARTProtocol)
            {
                case UART_APT:
                    LCD_APT_Task(&LCD_APT_handle); //Run the APT task
                    break;
                case UART_KD718:
                    LCD_KD718_Task(&LCD_KD718_handle); //Run the KD718 task
                    break;
                case UART_CLOUD_5S:
                    LCD_Cloud_5S_Task(&LCD_Cloud_5S_handle); //Run the Cloud 5S task
                    break;               
                case UART_LOG_HS:
                    LogHS_ProcessFrame(&LogHS_handle);
                    break;
                default:
                    break;
            }
            
            uCAL_UART_ClearTaskFlag(&UART0Handle);
            
            if (UART0Handle.UARTProtocol != UART_DISABLE)
            {
                VC_Errors_ClearError(SCREEN_COMM_ERROR);
            }                
        }
        else // We reached the timeout
        {
            switch(UART0Handle.UARTProtocol)
            {
                case UART_CLOUD_5S:
                    Throttle_UpdateExternal(pVCI->pPowertrain->pThrottle, 0); // Make sure we set the throttle to 0
                    PWRT_ForceDisengageCruiseControl(pVCI->pPowertrain);      // Make sure we exit cruise control if we were using it when we lost connection
                
                case UART_KD718:
                case UART_APT:
                    
                    VC_Errors_RaiseError(SCREEN_COMM_ERROR,HOLD_UNTIL_CLEARED);
                    break;
                case UART_LOG_HS:
                case UART_DISABLE:
                    break;                    
            }                
        }              
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
     * of CO_TIMER_INTERVAL in ms.
     */
    uint32_t ticks;
    ticks = COTmrGetTicks(&CONodeGNR.Tmr, CO_TIMER_INTERVAL, (uint32_t)CO_TMR_UNIT_1MS);
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
    if (ENABLE_IOT_BOARD != 0)
    {
        return true;
    }
    else
    {
        return false;
    }   
}

/**
  * Initialises the object dictionary with the user config values
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
    
        //CAN termination resistor enable status variable
        uint8_t configCANresistorStatus = UserConfigTask_GetCANterminatorState();
       
        /***************Throttle/Pedal Assist variables******************************/
        uint8_t maxPAS                             = UserConfigTask_GetNumberPasLevels();
        uint8_t pasMaxTorqueRatio                        = UserConfigTask_GetPasMaxTorqueRatio();
        
        uint16_t torqueSensorMultiplier[9] = {UserConfigTask_GetTorqueSensorMultiplier(PAS_1),
                                              UserConfigTask_GetTorqueSensorMultiplier(PAS_2),UserConfigTask_GetTorqueSensorMultiplier(PAS_3),
                                              UserConfigTask_GetTorqueSensorMultiplier(PAS_4),UserConfigTask_GetTorqueSensorMultiplier(PAS_5),
                                              UserConfigTask_GetTorqueSensorMultiplier(PAS_6),UserConfigTask_GetTorqueSensorMultiplier(PAS_7),
                                              UserConfigTask_GetTorqueSensorMultiplier(PAS_8),UserConfigTask_GetTorqueSensorMultiplier(PAS_9)};

        uint8_t pasLevelMinTorque[9]       = {UserConfigTask_GetPasLevelMinTorque(PAS_1),
                                              UserConfigTask_GetPasLevelMinTorque(PAS_2), UserConfigTask_GetPasLevelMinTorque(PAS_3),
                                              UserConfigTask_GetPasLevelMinTorque(PAS_4), UserConfigTask_GetPasLevelMinTorque(PAS_5),
                                              UserConfigTask_GetPasLevelMinTorque(PAS_6), UserConfigTask_GetPasLevelMinTorque(PAS_7),
                                              UserConfigTask_GetPasLevelMinTorque(PAS_8), UserConfigTask_GetPasLevelMinTorque(PAS_9)};
    
        uint8_t PasLevelSpeed[9]           = {UserConfigTask_GetPasLevelSpeed(PAS_1),
                                              UserConfigTask_GetPasLevelSpeed(PAS_2),UserConfigTask_GetPasLevelSpeed(PAS_3),
                                              UserConfigTask_GetPasLevelSpeed(PAS_4),UserConfigTask_GetPasLevelSpeed(PAS_5),
                                              UserConfigTask_GetPasLevelSpeed(PAS_6),UserConfigTask_GetPasLevelSpeed(PAS_7),
                                              UserConfigTask_GetPasLevelSpeed(PAS_8),UserConfigTask_GetPasLevelSpeed(PAS_9)};

        uint8_t pasLevelMaxTorque[9]       = {UserConfigTask_GetPasLevelMaxTorque(PAS_1),
                                              UserConfigTask_GetPasLevelMaxTorque(PAS_2),UserConfigTask_GetPasLevelMaxTorque(PAS_3),
                                              UserConfigTask_GetPasLevelMaxTorque(PAS_4),UserConfigTask_GetPasLevelMaxTorque(PAS_5),
                                              UserConfigTask_GetPasLevelMaxTorque(PAS_6),UserConfigTask_GetPasLevelMaxTorque(PAS_7),
                                              UserConfigTask_GetPasLevelMaxTorque(PAS_8),UserConfigTask_GetPasLevelMaxTorque(PAS_9)};
        
        uint8_t  configPasOverThrottle       = UserConfigTask_GetPASOverThrottle();
                                              
        uint8_t  configMaxSpeed              = UserConfigTask_GetBikeMaxSpeed();
        uint8_t  configWalkModeSpeed         = UserConfigTask_GetWalkmodeSpeed(); 
        uint8_t  configWalkmodeMaxTorque     = UserConfigTask_GetWalkmodeMaxTorque();
        uint8_t  configWalkmodeAccelRampType = UserConfigTask_GetWalkmodeAccelRampType();
        uint16_t configWalkmodeAccelRampArg1 = UserConfigTask_GetWalkmodeAccelRampArg1();

        uint16_t configPasStartupSpeedThreshold                 = UserConfigTask_GetPASStatupSpeedThreshold();
        uint16_t configPasRuntimeSpeedThreshold                 = UserConfigTask_GetPASRuntimeSpeedThreshold();
        uint16_t configPasTorqueDetectionValue                  = UserConfigTask_GetPASTorqueDetectionValue();
        uint16_t configPasStartupPulses                         = UserConfigTask_GetPASStartupPulses();
        uint16_t configPasStartupTimeWindow                     = UserConfigTask_GetPASStartupTimeWindow();
        uint16_t configPasRuntimePulses                         = UserConfigTask_GetPASRuntimePulses();
        uint16_t configPasRuntimeTimeWindow                     = UserConfigTask_GetPASRuntimeTimeWindow();
        uint8_t  configPasCadenceAndOrTorque                    = UserConfigTask_GetPASCadenceAndOrTorque();
        uint8_t  configPasTorqueScalingPedalRPM                 = UserConfigTask_GetPASTorqueScalingPedalRPMActivated();
        uint16_t configPasTorqueScalingMinRPM                   = UserConfigTask_GetPASTorqueScalingMinRPM();
        uint16_t configPasTorqueScalingMaxRPM                   = UserConfigTask_GetPASTorqueScalingMaxRPM();
        uint16_t configPasTorqueScalingMinRPMGain               = UserConfigTask_GetPASTorqueScalingMinRPMGain();
        uint16_t configPasTorqueScalingMaxRPMGain               = UserConfigTask_GetPASTorqueScalingMaxRPMGain();
        uint8_t  configPasRampType                              = UserConfigTask_GetPASRampType();
        uint16_t configDynamicDecelerationRampStart             = UserConfigTask_GetDynamicDecelerationRampStart();
        uint16_t configDynamicDecelerationRampEnd               = UserConfigTask_GetDynamicDecelerationRampEnd();
        uint16_t configDynamicDecelerationRampMinDeceleration   = UserConfigTask_GetDynamicDecelerationMinDeceleration();
        uint16_t configDynamicDecelerationRampMaxDeceleration   = UserConfigTask_GetDynamicDecelerationMaxDeceleration();
        uint16_t configHighSpeedPowerLimitingRampStart          = UserConfigTask_GetHighSpeedPowerLimitingRampStart();
        uint16_t configHighSpeedPowerLimitingRampEnd            = UserConfigTask_GetHighSpeedPowerLimitingRampEnd();
        uint16_t configHighSpeedPowerLimitingRampMinSpeedPower  = UserConfigTask_GetHighSpeedPowerLimitingMinSpeedPower();
        uint16_t configHighSpeedPowerLimitingRampMaxSpeedPower  = UserConfigTask_GetHighSpeedPowerLimitingMaxSpeedPower();
                                              
        //Config    
        uint8_t  configPasNbMagnetsPerTurn = UserConfigTask_GetPasNbMagnetsPerTurn();                                     
        uint16_t configPasTorqueInputMax = UserConfigTask_GetPasTorqueInputMax();
        uint16_t configPasTorqueInputMin = UserConfigTask_GetPasTorqueInputMin();
        
        uint16_t configBatteryFullVoltage  = UserConfigTask_GetBatteryFullVoltage();
        uint16_t configBatteryEmptyVoltage = UserConfigTask_GetBatteryEmptyVoltage();
                                              
        uint8_t configWheelSpeedSensorNbrMagnets = UserConfigTask_GetWheelSpeedSensorNbrMagnets();
        uint8_t configWheelDiameter  =  UserConfigTask_GetWheelDiameter();
        uint8_t configScreenProtocol =  UserConfigTask_GetScreenProtocol();  

        bool configisMotorMixedSignal = UserConfigTask_GetMotorMixedSignalState(); 
        uint16_t configMinSignalThreshold = UserConfigTask_GetMinSignalThreshold();
        uint32_t configMaxWheelSpeedPeriodUs = UserConfigTask_GetMaxWheelSpeedPeriodUs();
         
        uint16_t configBatteryMaxPeakDCCurrent              = UserConfigTask_GetBatteryMaxPeakDCCurrent();
        uint16_t configBatteryContinuousDCCurrent           = UserConfigTask_GetBatteryContinuousDCCurrent();        
        uint16_t configBatteryPeakCurrentMaxDuration        = UserConfigTask_GetBatteryPeakCurrentMaxDuration();
        uint16_t configBatteryPeakCurrentDeratingDuration   = UserConfigTask_GetBatteryPeakCurrentDeratingDuration();                                      
                                              
        uint8_t configHeadLightDefault      = UserConfigTask_GetHeadLightDefault(); 
        
        uint8_t configTailLightDefault      = UserConfigTask_GetTailLightDefault();
        uint8_t configTailLightBlinkOnBrake = UserConfigTask_GetTailLightBlinkOnBrake();
                                              
        uint16_t configThrottleAdcOffset    = UserConfigTask_GetThrottleAdcOffset();                                      
        uint16_t configThrottleAdcMax       = UserConfigTask_GetThrottleAdcMax();
        uint8_t  configThrottleBlockOff     = UserConfigTask_GetThrottleBlockOff();
        uint8_t  configThrottleMaxSpeed     = UserConfigTask_GetThrottleMaxSpeed();
        uint8_t  configThrottleAccelRampType = UserConfigTask_GetThrottleAccelRampType();
        uint16_t configThrottleAccelRampArg1 = UserConfigTask_GetThrottleAccelRampArg1();
                                              
        
        uint8_t  FilterSpeed[FILTERSPEED_ARRAY_SIZE] = {UserConfigTask_GetFilterSpeed(0), UserConfigTask_GetFilterSpeed(1)}; 
        uint16_t pasLowPassFilterBW1[BW_ARRAY_SIZE] = {UserConfigTask_GetFilterBwValue(0, BW1), UserConfigTask_GetFilterBwValue(1, BW1), UserConfigTask_GetFilterBwValue(2, BW1)};
        uint16_t pasLowPassFilterBW2[BW_ARRAY_SIZE] = {UserConfigTask_GetFilterBwValue(0, BW2), UserConfigTask_GetFilterBwValue(1, BW2), UserConfigTask_GetFilterBwValue(2, BW2)};
        
        uint8_t motorTempSensorType = UserConfigTask_GetMotorSensorType();
        uint16_t motorNTCBetaCoef = UserConfigTask_GetMotorNTCBetaCoef();
        uint16_t motorNTCResistanceCoef = UserConfigTask_GetMotorNTCResistanceCoef();
          
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SERIAL_NB, M2)),     pNode, &fSerialNbLow, sizeof(fSerialNbLow));     
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SERIAL_NB, M1)),     pNode, &fSerialNbHigh,  sizeof(fSerialNbHigh));  
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FW_VERSION, M1)),    pNode, &hFwVersion, sizeof(uint32_t));           
        
        /***********************UPdate Throttle/Pedal Assist CANopen OD ID.********************************************/     
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_PAS, 0)),               pNode, &maxPAS, sizeof(uint8_t));            
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MAX_TORQUE_RATIO, 0)),  pNode, &pasMaxTorqueRatio, sizeof(uint8_t));             
        
        // Initialise the number of magnets for the wheel speed sensor with the value in the user config 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 1)),       pNode, &configWheelDiameter, sizeof(uint8_t));
        // Initialise the wheel diameter with the value in the user config 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 0)),       pNode, &configWheelDiameter, sizeof(uint8_t));
        
        //Initialise isMotorMixedSignal with the value in the user config
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 4)), pNode, &configisMotorMixedSignal, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 5)), pNode, &configMinSignalThreshold, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 6)), pNode, &configMaxWheelSpeedPeriodUs, sizeof(uint32_t));
        
        // Initialise the light with the default value in user config
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0)),   pNode, &configHeadLightDefault, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 0)),    pNode, &configTailLightDefault, sizeof(uint8_t));
        
        //fill the OD ID to PasLevelSpeed and pasLevelMaxTorque with the current values.
        //this OD ID have 10 subindex each.
        for(uint8_t n = 0; n < 9; n++)                                                                                                      
        {
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_TORQUE_SENSOR_MULTIPLIER, n)), pNode, &torqueSensorMultiplier[n], sizeof(uint16_t));
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_LEVEL_SPEED, n)), pNode, &PasLevelSpeed[n], sizeof(uint8_t)); 
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MIN_TORQUE, n)),  pNode, &pasLevelMinTorque[n], sizeof(uint8_t)); 
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_PAS_MAX_TORQUE, n)),  pNode, &pasLevelMaxTorque[n], sizeof(uint8_t));                     
        }

        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 8)), pNode, &configPasOverThrottle, sizeof(uint8_t)); 
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MAX_SPEED, 0)),       pNode, &configMaxSpeed, sizeof(uint8_t));                                 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 0)), pNode, &configWalkModeSpeed, sizeof(uint8_t)); 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 1)), pNode, &configWalkmodeMaxTorque, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 2)), pNode, &configWalkmodeAccelRampType, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WALK_MODE_SPEED, 3)), pNode, &configWalkmodeAccelRampArg1, sizeof(uint16_t));
        
        //Config 

        // Subindex 0-2 have place holder but have not been implemented yet.
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 3)),       pNode, &configPasNbMagnetsPerTurn, sizeof(uint8_t));  
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 4)),       pNode, &configPasTorqueInputMin, sizeof(uint16_t));  
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(COD_OD_REG_PAS_SENSOR, 5)),       pNode, &configPasTorqueInputMax, sizeof(uint16_t));  
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_VOLTAGE, 0)),        pNode, &configBatteryFullVoltage, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_VOLTAGE, 1)),        pNode, &configBatteryEmptyVoltage, sizeof(uint16_t));
        
        // Show what is the default number of magnets for the wheel speed sensor in the user config 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 1)),        pNode, &configWheelSpeedSensorNbrMagnets, sizeof(uint8_t));         
        // Show what is the default wheel diameter in the user config 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_WHEELS, 2)),        pNode, &configWheelDiameter, sizeof(uint8_t));         
        // Show the currently selected screen protocol
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_SCREEN_PROTOCOL, 0)),     pNode, &configScreenProtocol, sizeof(uint8_t)); 
        
        // Headlight default state
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 0)),    pNode, &configHeadLightDefault, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_FRONT_LIGHT, 1)),    pNode, &configHeadLightDefault, sizeof(uint8_t));
        // Taillight default state and blink on brake state
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 0)),     pNode, &configTailLightDefault, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 1)),     pNode, &configTailLightDefault, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_VEHICLE_REAR_LIGHT, 2)),     pNode, &configTailLightBlinkOnBrake, sizeof(uint8_t));
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 0)),     pNode, &configBatteryMaxPeakDCCurrent, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 1)),     pNode, &configBatteryContinuousDCCurrent, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 2)),     pNode, &configBatteryPeakCurrentMaxDuration, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_BATTERY_DC_CURRENT, 3)),     pNode, &configBatteryPeakCurrentDeratingDuration, sizeof(uint16_t));
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 2)),    pNode, &configThrottleAdcOffset, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 3)),    pNode, &configThrottleAdcMax, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 4)),    pNode, &configThrottleBlockOff, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 5)),    pNode, &configThrottleMaxSpeed, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 6)),    pNode, &configThrottleAccelRampType, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CONTROLLER_THROTTLE, 7)),    pNode, &configThrottleAccelRampArg1, sizeof(uint16_t));

        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_THRESHOLD, 0)),        pNode, &configPasStartupSpeedThreshold, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_SPEED_THRESHOLD, 1)),        pNode, &configPasRuntimeSpeedThreshold, sizeof(uint16_t));
                                 
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 0)),   pNode, &configPasTorqueDetectionValue, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 1)),   pNode, &configPasStartupPulses, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 2)),   pNode, &configPasStartupTimeWindow, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 3)),   pNode, &configPasRuntimePulses, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_DETECTION_PARAMETERS, 4)),   pNode, &configPasRuntimeTimeWindow, sizeof(uint16_t));
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CADENCE_AND_OR_TORQUE, 0)),      pNode, &configPasCadenceAndOrTorque, sizeof(uint8_t));

        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 0)),   pNode, &configPasTorqueScalingPedalRPM, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 1)),   pNode, &configPasTorqueScalingMinRPM, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 2)),   pNode, &configPasTorqueScalingMaxRPM, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 3)),   pNode, &configPasTorqueScalingMinRPMGain, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_TORQUE_SCALING_PEDAL_RPM, 4)),   pNode, &configPasTorqueScalingMaxRPMGain, sizeof(uint16_t));

        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_PAS_RAMP_SELECTION, 0)),         pNode, &configPasRampType, sizeof(uint8_t));
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 0)), pNode, &configDynamicDecelerationRampStart, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 1)), pNode, &configDynamicDecelerationRampEnd, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 2)), pNode, &configDynamicDecelerationRampMaxDeceleration, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_DYNAMIC_DECELERATION_RAMP_PARAMS, 3)), pNode, &configDynamicDecelerationRampMinDeceleration, sizeof(uint16_t));
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 0)), pNode, &configHighSpeedPowerLimitingRampStart, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 1)), pNode, &configHighSpeedPowerLimitingRampEnd, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 2)), pNode, &configHighSpeedPowerLimitingRampMinSpeedPower, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_HIGH_SPEED_POWER_LIMITER_PARAMS, 3)), pNode, &configHighSpeedPowerLimitingRampMaxSpeedPower, sizeof(uint16_t));
         
        //torque band filter parameters
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER, 0)),    pNode, &FilterSpeed[0], sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER, 1)),    pNode, &FilterSpeed[1], sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 0)),    pNode, &pasLowPassFilterBW1[0], sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 1)),    pNode, &pasLowPassFilterBW2[0], sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 2)),    pNode, &pasLowPassFilterBW1[1], sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 3)),    pNode, &pasLowPassFilterBW2[1], sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 4)),    pNode, &pasLowPassFilterBW1[2], sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED, 5)),    pNode, &pasLowPassFilterBW2[2], sizeof(uint16_t));
        
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMPERATURE, 0)),    pNode, &motorTempSensorType, sizeof(uint8_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMPERATURE, 1)),    pNode, &motorNTCBetaCoef, sizeof(uint16_t));
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_MOTOR_TEMPERATURE, 2)),    pNode, &motorNTCResistanceCoef, sizeof(uint16_t));
        
        //Can Resistor parameter
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_CAN_TERMINAL_RESISTOR, 0)), pNode, &configCANresistorStatus, sizeof(uint8_t));
        
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