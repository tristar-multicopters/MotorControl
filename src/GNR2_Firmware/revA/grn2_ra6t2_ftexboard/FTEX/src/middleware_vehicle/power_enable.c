/**
  * @file    power_enable.c
  * @brief   This module handles power enable pin
  *
*/

#include "power_enable.h"
#include "vc_config.h"
#include "board_hardware.h"
#include "gnr_main.h"
#include "firmware_update.h"

extern osThreadId_t PowerOffSequence_handle;

static PWREN_PowerOffSequencyState_t PWREN_PowerOffSequencyState = PWREN_IDLE;

/**
  Function used to initialise the GPIO pins used for the power enable module
*/
void PWREN_Init(PWREN_Handle_t * pHandle)
{    
    ASSERT(pHandle != NULL);

    struct GPIOConfig PinConfig;
   
    PinConfig.PinDirection = INPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(PWR_ENABLE_GPIO_PIN, PinConfig);
   
    PinConfig.PinDirection = OUTPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(CAN_STANDBY_N_GPIO_PIN, PinConfig);
    
    if (PWREN_IsPowerEnabled(pHandle)) 
    { 
        pHandle->bInitialPowerLockState = true;   
    }
    else
    {
        pHandle->bInitialPowerLockState = false;   
    }
    
    //initialize system ready timeout count.
    pHandle->bSytemReadyTimeout = 0;
    
    //PWREN struct was initialized.
    pHandle->bInitalized = true;
}

/**
  Function used to check the state of the Power Enable signal
*/
bool PWREN_IsPowerEnabled(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if (pHandle->bUsePowerLock)
    {          
        pHandle->bIsPowerEnabled = uCAL_GPIO_Read(PWR_ENABLE_GPIO_PIN) ^ pHandle-> bIsInvertedLogic;
        
        return pHandle->bIsPowerEnabled;
    }
    else
    {
        return true;
    }       
}

/**
   Function used to monitor the current state of the Power Enable signal 
   and trigger the power off sequence if needed
*/
void PWREN_MonitorPowerEnable(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    // test if display resquested the turn off.
    if((PWREN_IsPowerEnabled(pHandle) == false) && (pHandle->bInitialPowerLockState == true))        
    {   
        osThreadFlagsSet(PowerOffSequence_handle,POWEROFFSEQUENCE_FLAG);
    }
    
    //test if was false turn on command received by CAN interface 
    if ((PWREN_IsPowerEnabled(pHandle) == false) &&(pHandle->bWakeUpCommandChecked == true) && (pHandle->bWakeUpSDOCommand == false))
    {
        
        osThreadFlagsSet(PowerOffSequence_handle,POWEROFFSEQUENCE_FLAG);
    }
} 

/**
   Function used to turn off the power to the controller
*/
void PWREN_StopPower(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);

    uCAL_GPIO_Reset(CAN_STANDBY_N_GPIO_PIN);
}

/**
  @brief Function to check if the firmware update command was received.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param uint8_t UpdateCommand used to pass the value read from the CAN dictionary 
         address responsible to hold the firmware update command.
  @return void
*/
void PWREN_CheckFirmwareUpdateCommand(PWREN_Handle_t * pHandle, uint8_t UpdateCommand)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);
    
    //variable used to check the timeout to receive a firmware update command.
    static uint16_t firmwareUpdateTimeout = 0;   

    //Start to check if the firmware update command was received.\
    //This command afect the way the PWREM works(power on/off management).
    //If the PWREN was initialized the firmware update check request can be made.
    if((pHandle->bInitalized == true) && (pHandle->bWakeUpSDOCommand == false))
    {     
        //Check if the the device received a wake up command by SDO to start the bootloader mode(firmware update).
        if(UpdateCommand == FIRMWARE_UPDATE_REQUEST)
        {
            //Set the flag to say if the command was received or a timeout happened.
            //This decision will be made by the PWREN.
            // bWakeUpSDOCommand == true and bWakeUpSDOCommandChecked == true
            //means command was received.
            pHandle->bWakeUpCommandChecked = true;
                    
            //Set the the correspondent flag to indicate that a firmware update request arrived.
            pHandle->bWakeUpSDOCommand = true; 
        }
                
        //Check if the device was woken up by the a CAN event.
        //if was start to count the timeout.                
        if(pHandle->bInitialPowerLockState == false)
        {
            //
            firmwareUpdateTimeout++;
        }
                
        //if in 5 seconds the device don't receive a firmware update command, he will set
        //the flag above.
        if((firmwareUpdateTimeout > FIRMWAREUPDATE_TIMEOUT) && (pHandle->bWakeUpCommandChecked == false))
        {
            //Set the flag to say if the command was received or a timeout happened.
            //This decision will be made by the PWREN.
            // bWakeUpSDOCommand == false and bWakeUpSDOCommandChecked == true
            //means command was not received.
            pHandle->bWakeUpCommandChecked = true;
        } 
    }
}

/**
  @brief Function used to turn off the slaver motor, if it's present,
         and inform IOT module the device is going to turn off.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_TurnOffSystem(CO_NODE  *pNode, PWREN_Handle_t * pHandle)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);
    ASSERT(pNode != NULL);
    
    //Declare a pointer to parent CANopen node
    static CO_CSDO *csdo = NULL;
    
    //check if the display was turnned on again
    //if yes, stop the turn off sequency reseting
    //the device.
    if (PWREN_IsPowerEnabled(pHandle)) 
    { 
        NVIC_SystemReset();   
    }
    
    //state machine used to control the power off sequency of the system(iot+slvave+master).
    switch(PWREN_PowerOffSequencyState)
    {
        //wait for the goingoff flag to became true.
        //this means thta I power off was requested.
        case PWREN_IDLE:
            //verify if the device is enabled to start the turn off
            //sequency.
            if (pHandle->bGoingOff == true)
            {
                //if the device is a master + iot move to the state PWREN_STOPIOT
                //to stop IOT.
                #if (GNR_IOT == 1 && GNR_MASTER == 1)
                //move to PWREN_STOPIOT state to say to IOT module that the device is
                //going turn off.
                PWREN_PowerOffSequencyState = PWREN_STOPIOT;
                
                //prepare a value to confirm to IOT/Slave that the master is going turn off.
                pHandle->bSdoData = 0x01;
                
                //Initialize the number of attempts.
                pHandle->bSdoAttempts = 0;
                
                //set to false before send the sdo download command.
                pHandle->bIotStoped = false;
                
                //get the corresponding CO_CSDO object.
                csdo = COCSdoFind(pNode, SDOCLIENTINDEX_IOT);
    
                //verify if the SDO client is missing.
                ASSERT(csdo != NULL);
                
                #else
                //move to PWREN_TURNOFFSLAVE state to say to slave module that the device is
                //going turn off and slave must turn off yourself too.
                PWREN_PowerOffSequencyState = PWREN_TURNOFFSLAVE;
                
                //prepare a value to confirm to IOT/Slave that the master is going turn off.
                pHandle->bSdoData = 0x01;
                
                //Initialize the number of attempts.
                pHandle->bSdoAttempts = 0;
                
                //set to false before send the sdo download command.
                pHandle->bSlaveOff = false;
                
                //get the corresponding CO_CSDO object.
                csdo = COCSdoFind(pNode, SDOCLIENTINDEX_SLAVE);
    
                //verify if the SDO client is missing.
                ASSERT(csdo != NULL);
                
                #endif
            }
        break;
        
        //write on IOT to say that a turn off sequency is going.
        case PWREN_STOPIOT:
            
            //false before send a sdo msg.this is set to true when 
            //an automatic response(CANOPEN response) is generated.
            //system will know that an answer was received to confirme
            //the sent sdo msg.
            pHandle->bSdoResponseReceived = false;
        
            //Send a SDO donwload command to write in the OD of the IOT
            //if returns something different from CO_ERR_NONE sod failled.
            if (CO_ERR_NONE != COCSdoRequestDownload(csdo, CO_DEV(CO_OD_REG_DEVICE_TURNNING_OFF, 0), &pHandle->bSdoData, sizeof(pHandle->bSdoData), IOT_CallbackSDODownloadFinish, SDO_TIMEOUT))
            {
                //increment the number of attempts.
                pHandle->bSdoAttempts++;
                
                //waits(task on hold state) for 25ms to send again a sdo command. 
                osDelay(SDO_TIMEOUT);
            }
            else
            {
                //move to PWREN_STOPIOT state to say to IOT module that the device is
                //going turn off.
                PWREN_PowerOffSequencyState = PWREN_WAITIOT;
            }
            
            //verify if the maximum attempts's number was exceded.
            //if yes, move to turn off or turn off slave state.
            if (pHandle->bSdoAttempts > SDODOWNLOAD_MAXATTEMPTS)
            {
                #if (GNR_IOT == 1 && SUPPORT_SLAVE_ON_IOT == 1)
                PWREN_PowerOffSequencyState = PWREN_TURNOFFSLAVE;
                //prepare a value to confirm to IOT/Slave that the master is going turn off.
                pHandle->bSdoData = 0x01;
                
                //Initialize the number of attempts.
                pHandle->bSdoAttempts = 0;
                
                //set to false before send the sdo download command.
                pHandle->bSlaveOff = false;
                
                //get the corresponding CO_CSDO object.
                csdo = COCSdoFind(pNode, SDOCLIENTINDEX_SLAVE);
    
                //verify if the SDO client is missing.
                ASSERT(csdo != NULL);
                #else
                //move to turn off state.
                PWREN_PowerOffSequencyState = PWREN_TURNOFF;
                #endif
            }
        break;
        
        //state used to wait a confirmation or abort message from IOT.
        case PWREN_WAITIOT:
            
            //verify if the sdo msg was received or aborted and process it.
            //if not wait until receive a confirmation.
            if (pHandle->bSdoResponseReceived == true)
            {
                //verify if the sdo downalod message was received by the client.
                //if yes, moce to turn off the slave if it exist.
                if (pHandle->bIotStoped != true)
                {
                    //increment the number of attempts.
                    pHandle->bSdoAttempts++;
                
                    //move back to PWREN_STOPIOT state to try again.
                    PWREN_PowerOffSequencyState = PWREN_STOPIOT;
                }
                else
                {
                    //if the device is a master + iot supporting slave move to the state PWREN_TURNOFFSLAVE
                    //to stop SLAVE.
                    #if (GNR_IOT == 1 && SUPPORT_SLAVE_ON_IOT == 1)
                    PWREN_PowerOffSequencyState = PWREN_TURNOFFSLAVE;
                    //prepare a value to confirm to IOT/Slave that the master is going turn off.
                    pHandle->bSdoData = 0x01;
                
                    //Initialize the number of attempts.
                    pHandle->bSdoAttempts = 0;
                
                    //set to false before send the sdo download command.
                    pHandle->bSlaveOff = false;
                
                    //get the corresponding CO_CSDO object.
                    csdo = COCSdoFind(pNode, SDOCLIENTINDEX_SLAVE);
    
                    //verify if the SDO client is missing.
                    ASSERT(csdo != NULL);
                    #else
                    //move to turn off state.
                    PWREN_PowerOffSequencyState = PWREN_TURNOFF;
                    #endif
                }
                
                //verify if the maximum attempts's number was exceded.
                //if yes, move to turn off or turn off slave state.
                if (pHandle->bSdoAttempts > SDODOWNLOAD_MAXATTEMPTS)
                {
                    #if (GNR_IOT == 1 && SUPPORT_SLAVE_ON_IOT == 1)
                    PWREN_PowerOffSequencyState = PWREN_TURNOFFSLAVE;
                    //prepare a value to confirm to IOT/Slave that the master is going turn off.
                    pHandle->bSdoData = 0x01;
                
                    //Initialize the number of attempts.
                    pHandle->bSdoAttempts = 0;
                
                    //set to false before send the sdo download command.
                    pHandle->bSlaveOff = false;
                
                    //get the corresponding CO_CSDO object.
                    csdo = COCSdoFind(pNode, SDOCLIENTINDEX_SLAVE);
    
                    //verify if the SDO client is missing.
                    ASSERT(csdo != NULL);
                    #else
                    //move to turn off state.
                    PWREN_PowerOffSequencyState = PWREN_TURNOFF;
                    #endif
                }
            }
            
        break;
        
        //write on SLAVE to say that a turn off sequency is going.
        case PWREN_TURNOFFSLAVE:
            //false before send a sdo msg.this is set to true when 
            //an automatic response(CANOPEN response) is generated.
            //system will know that an answer was received to confirme
            //the sent sdo msg.
            pHandle->bSdoResponseReceived = false;
        
            //Send a SDO donwload command to write in the OD of the SLAVE.
            //if returns something different from CO_ERR_NONE sod failled.
            if (CO_ERR_NONE != COCSdoRequestDownload(csdo, CO_DEV(CO_OD_REG_DEVICE_TURNNING_OFF, 0), &pHandle->bSdoData, sizeof(pHandle->bSdoData), Slave_CallbackSDODownloadFinish, SDO_TIMEOUT))
            {
                //increment the number of attempts.
                pHandle->bSdoAttempts++;
                
                //waits(task on hold state) for 25ms to send again a sdo command. 
                osDelay(SDO_TIMEOUT);
            }
            else
            {
                //move to PWREN_STOPIOT state to say to SLAVE module that the device is
                //going turn off.
                PWREN_PowerOffSequencyState = PWREN_WAITTURNOFFSLAVE;
            }
            
            //verify if the maximum attempts's number was exceded.
            //if yes, move to turn off state.
            if (pHandle->bSdoAttempts > SDODOWNLOAD_MAXATTEMPTS)
            {
                //move to turn off state.
                PWREN_PowerOffSequencyState = PWREN_TURNOFF;
            }
        break;
        
        //state used to wait a confirmation or abort message from the slave.
        case PWREN_WAITTURNOFFSLAVE:
            //verify if the sdo msg was received or aborted and process it.
            //if not wait until receive a confirmation.
            if (pHandle->bSdoResponseReceived == true)
            {
                //verify if the sdo downalod message was received by the client.
                //if yes, move to turn off thr device.
                if (pHandle->bSlaveOff != true)
                {
                    //increment the number of attempts.
                    pHandle->bSdoAttempts++;
                
                    //move back to PWREN_STOPIOT state to try again.
                    PWREN_PowerOffSequencyState = PWREN_TURNOFFSLAVE;
                }
                else
                {
                    //move to turn off state.
                    PWREN_PowerOffSequencyState = PWREN_TURNOFF;
                }
                
                //verify if the maximum attempts's number was exceded.
                //if yes, move to turn off state.
                if (pHandle->bSdoAttempts > SDODOWNLOAD_MAXATTEMPTS)
                {
                    //move to turn off state.
                    PWREN_PowerOffSequencyState = PWREN_TURNOFF;
                }
            }
        break;
        
        //state where the device is turnned off.
        case PWREN_TURNOFF:
            //stop the node to stops CANOPEN msg on the can bus.
            CONodeStop(pNode);
        
            //wait 100 ms before turn off the device.
            //thsi give sometime to IOT and SLAVE
            //process the turn off sequency and avoid
            //new CANOPEN msgs.
            osDelay(POWEROFF_WAITTIME);
        
            //turn off himself 
            PWREN_StopPower(pHandle);
        
            //rtos delay
            //if after this delay the device still on
            //power off sequency was not done correctly
            //and the device must do a software reset.
            //*sometimes this happene because the canstdby
            //signal, controlled by the PWREN_StopPower() 
            //function, turn off the power management modulo
            //and microcontroller still on and the screen turn it
            //on. THis situation let the device inside of the 
            //power off task, blocking a lot of resources.
            osDelay(POWEROFF_ERRORTIMEOUT_MS);
        
            //if device still on, reset the system.
            NVIC_SystemReset();
        break;   
    }
}

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to IOT module.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void IOT_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    UNUSED_PARAMETER(csdo);
    UNUSED_PARAMETER(index);
    UNUSED_PARAMETER(sub);
    //if code is zero sdo download was correctly wrote on the client node(OD).
    if (code == 0)
    {
        //set to true to indicate that sdo services was received and execuated by
        //the client.
        VCInterfaceHandle.pPowertrain->pPWREN->bIotStoped = true;   
    }
    
    //sdo feedback/confirmation was received.
    VCInterfaceHandle.pPowertrain->pPWREN->bSdoResponseReceived = true;
}

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to SLAVE module.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void Slave_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    UNUSED_PARAMETER(csdo);
    UNUSED_PARAMETER(index);
    UNUSED_PARAMETER(sub);
    
    //if code is zero sdo download was correctly wrote on the client node(OD).
    if (code == 0)
    {
        //set to true to indicate that sdo services was received and execuated by
        //the client.
        VCInterfaceHandle.pPowertrain->pPWREN->bSlaveOff = true;   
    }
    
    //sdo feedback/confirmation was received.
    VCInterfaceHandle.pPowertrain->pPWREN->bSdoResponseReceived = true;
}

/**
  @brief Function used to set the going off flag.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_SetGoingOffFlag(PWREN_Handle_t * pHandle)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);
    
    //set the goingoff flag.
    pHandle->bGoingOff = true;
}

/**
  @brief Function used to get the value of the going off flag.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return bool return the going off flag value.
*/
bool PWREN_GetGoingOffFlag(PWREN_Handle_t * pHandle)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);

    //set the goingoff flag.
    return pHandle->bGoingOff;
}

/**
  @brief Function used to wait the system be ready to start can open communication
         This function set a flag that control the ready or not state of the CANOPEN
         communication and say if the system can do a normal power off sequency or not.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_ManageSystemReadyFlag(CO_NODE  *pNode, PWREN_Handle_t * pHandle)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);
    ASSERT(pNode != NULL);
    
    //slave is not using bInitalized, only master.
    #if GNR_MASTER
    //wait for the pwr enable initialization
    //if the pwr lock signal was checked 
    //start the timeout count to set the
    //flag system ready.
    if ((pHandle->bInitalized == true) && ( pHandle->bSystemReady == false))
    {
    #else
    if (pHandle->bSystemReady == false)
    {
    #endif
        //increment system ready timeout.
        pHandle->bSytemReadyTimeout++;
        
        //cehck if timeout
        if (pHandle->bSytemReadyTimeout > SYSTEMREADY_TIMEOUT)
        {   
            //Start the CANopen node and set it automatically to
            //NMT mode: 'OPERATIONAL'.
            CONodeStart(pNode);
            CONmtSetMode(&pNode->Nmt, CO_OPERATIONAL);
            
            //set the system ready flag.
            pHandle->bSystemReady = true;
            
            //initiliaze the timeout variable to be used on
            //PWREN_SetIotSystemIsOn(this function will be called
            //only when PWREN_ManageSystemReadyFlag has finished).
            //doing this to save RAM memory.
            pHandle->bSytemReadyTimeout = 0;
        }
    } 
}

/**
  @brief Function used to change the IOT OD inform him that GNR is on.
         communication and say if the system can do a normal power off sequency or not.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return void
*/
void PWREN_SetIotSystemIsOn(CO_NODE  *pNode, PWREN_Handle_t * pHandle)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);
    ASSERT(pNode != NULL);
    
    //if GNR is not going off(turn off procedure) or if the system
    //did start a firmware update move in to try to send a new sdo
    //msg if the period match.
    if ((pHandle->bGoingOff == false) && (FirmwareUpdate_Running() == false))
    {
        //increment timeout to send the next sdo download 
        //to IOT module.
        pHandle->bSytemReadyTimeout++;
        
        //check if is time to send a msg to the IOT module.
        //timeout is 1000ms.
        if (pHandle->bSytemReadyTimeout >= IOTMSG_PERIOD)
        {   
            //initiliaze the timeout variable
            //to prepare the next period.
            pHandle->bSytemReadyTimeout = 0;
            
            //Declare a pointer to parent CANopen node
            CO_CSDO *csdo;
            
            //get the corresponding CO_CSDO object.
            csdo = COCSdoFind(pNode, SDOCLIENTINDEX_IOT);
            
            //set to zero(system on).
            pHandle->bSdoData = 0x00;
            
            //send a sdo download msg
            COCSdoRequestDownload(csdo, CO_DEV(CO_OD_REG_DEVICE_TURNNING_OFF, 0), &pHandle->bSdoData, sizeof(pHandle->bSdoData), GnrOn_CallbackSDODownloadFinish, SDO_TIMEOUT);
        }
    } 
}

/**
  @brief Function used to get the system ready flag.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @return bool return the system ready value.
*/
bool PWREN_GetSystemReadyFlag(PWREN_Handle_t * pHandle)
{
    //verify if the pointer is null.
    ASSERT(pHandle != NULL);
    
    //set the goingoff flag.
    return pHandle->bSystemReady;
}

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to inform OIT module
         that the GNR is on.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void GnrOn_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    UNUSED_PARAMETER(csdo);
    UNUSED_PARAMETER(index);
    UNUSED_PARAMETER(sub);
    UNUSED_PARAMETER(code);
}

#if !GNR_MASTER

/**
  @brief Task used to allow a slave device turn off himself.
*/
__NO_RETURN void PWREN_TurnoffSlaveTask (void * pvParameter)
{
    //UNUSED_PARAMETER(pvParameter);
    CO_NODE  *pNode;
    pNode = (CO_NODE *)pvParameter;
    
    while(true)
    {
        //
        uint8_t turnningOff = 0;
        //critical section
        //Read the OD responsible to hold the firmware update command.
        COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_DEVICE_TURNNING_OFF, 0)), pNode, &turnningOff, sizeof(uint8_t));
        //end critical section;
        
        //test if a turn off command was received
        //this command is write in the OD of teh device 
        //and send by a master device.
        if (turnningOff > 0)
        {
            //small delay(75ms) to give time to the slave answer back 
            osDelay(SDO_TIMEOUT*3);
            
            //stop the node to stops CANOPEN msg on the can bus.
            CONodeStop(pNode);
        
            //turn off himself 
            PWREN_StopPower(VCInterfaceHandle.pPowertrain->pPWREN);
        }
    }
}

#endif
