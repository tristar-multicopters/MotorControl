/**
*  @file vc_fault_management.c
*  @brief middleware layer used to management VC faults.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "vc_fault_management.h"

/*********************************************
                Defines
*********************************************/

/*********************************************
                Private Variables
*********************************************/
//variable used to indicate a continue communication
//between master and slave.
static bool MasterSlaveDetectionFlag = false;

/*********************************************
                Public Variables
*********************************************/


/****************************************************************
                Private Functions Prototype
*****************************************************************/

                                                        

/****************************************************************
                Private Functions 
*****************************************************************/


/****************************************************************
                Public Functions
*****************************************************************/

/**
 *@brief Function used to set the presence of the device, master or slave.
 *       It send a sdo download command and if the other side responde a flag
 *       is set to indicate that command was received.
 *       No sdo response, clear the correspondent flag(MasterSlaveDetectionFlag).
*/
void VCFaultManagment_MasterSlaveDetection(CO_NODE  *pNode, PWREN_Handle_t * pHandle)
{
    ASSERT(pNode != NULL);
    ASSERT(pHandle != NULL);
    
    //variable used to hold the value to be write by sdo download.
    uint8_t sdoData = 0x01;
    
    //variable used to count until the right time to send the sdo download.
    static uint8_t countTimeSdo = 0;
    
    //Declare a pointer to parent CANopen node
    static CO_CSDO *csdo = NULL;
    
    //if master or master +IOT supporting slave the client index
    //will be SDOCLIENTINDEX_SLAVE. 
    //This is used to know in what GNR the device will righ(master or slave).
    //If master , it must write on slave, if slave it msut write on master.
    //Before read or write something on one server is necessary to get
    //server address(ID).
    if ((GNR_IOT && SUPPORT_SLAVE_ON_IOT) || (!GNR_IOT && VcAutodeter_GetGnrState()))
    {
        //get the corresponding CO_CSDO object(server id).
        csdo = COCSdoFind(pNode, SDOCLIENTINDEX_SLAVE);
    }
    else
    {
        //verify if the GNR is slave.
        if ((!GNR_IOT && !VcAutodeter_GetGnrState()))
        {
            //get the corresponding CO_CSDO object(server id).
            csdo = COCSdoFind(pNode, SDOCLIENTINDEX_MASTER);
        }
    }
    
    //check if csdo object is not NULL and if GNR is going on turn process.
    //this check if necessary to verify if the system is on single or dual motor.
    //only dual, master/slave, can send sdo download on the address CO_OD_REG_MASTER_SLAVE_PRESENT.
    if ((csdo != NULL) && (pHandle->bGoingOff == false))
    {
        //verify if the necessary time was reached to send the sdo download command.
        if (countTimeSdo >= MASTERSLAVE_DETECTION_TIME_250MS)
        {
            //Initialize send timeout.
            countTimeSdo = 0;
  
            //send the sdo command to the client(master or slave on this case).
            COCSdoRequestDownload(csdo, CO_DEV(CO_OD_REG_MASTER_SLAVE_PRESENT, 0), &sdoData, sizeof(sdoData), MasterSlaveDetection_CallbackSDODownloadFinish, SDODOWNLOAD_TIMEOUT_MS);
        }
        else
        {
            //Increment the time to send the next sdo download command.
            countTimeSdo++;
        }
    }
}

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to master/slave module.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void MasterSlaveDetection_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    UNUSED_PARAMETER(csdo);
    UNUSED_PARAMETER(index);
    UNUSED_PARAMETER(sub);
    //if code is zero sdo download was correctly wrote on the client node(OD).
    if (code == 0)
    {
        //set flag to indicate master/slave answered the sdo download.
        MasterSlaveDetectionFlag = true;
    }
    else
    {
        //clear flag to indicate master/slave didn't answer the sdo download.
        MasterSlaveDetectionFlag = false;
    }
}

/**
  @brief Function used to detect if slave or master are not present anymore.
         This means more than 0.5 seconds without communication. 
         This function is verified every 25 ms.
*/
bool VCFaultManagment_MasterSlaveCommunicationLost(void)
{
    //variable used to count until the maximum timeout without communication
    static uint8_t countNoDetectionTime = 0;
    
    //verify if the MasterSlaveDetectionFlag is false.
    //False means the communication between master/slave
    //was lost. If communication was lost, start timeout
    //count.
    if (!MasterSlaveDetectionFlag)
    {
        //verify if the maximum time without communication between master/slave was reached.
        if (countNoDetectionTime >= MASTERSLAVE_LOSTDETECTION_TIMEOUT_500MS)
        {            
            //return true to confirm lost of communication between master/slave.
            return true;
        }
        else
        {
            //Increment the time.
            countNoDetectionTime++;
        }
    }
    else
    {
       //Initialize timeout variable.
       countNoDetectionTime = 0;
        
       //return false to confirm communication between master/slave.
       return false; 
    }
    
    //this function arrive here when couting timeout.
    return false;
}

/**
  @brief Function used to process faults happened in the VC layer.
         If fault condition finished, clear fault error on vc layer.
*/
void VCFaultManagment_Processing(VCSTM_Handle_t *pHandle, CO_NODE  *pNode)
{
    ASSERT(pHandle != NULL);
    ASSERT(pNode != NULL);
    
    //verify what kind of fault was triggered.
    if (pHandle->hVFaultNow & VC_START_TIMEOUT)
    {
        //clear VC_START_TIMEOUT error.     
        //the idea is clear start timeout fault and make the
        //system back to idle state again and try to run again.
        VCSTM_FaultProcessing(pHandle, 0, VC_START_TIMEOUT);
    }
    
    //verify if was a lost of communication with
    //slaver.
    if (pHandle->hVFaultNow & VC_SLAVE_COMM_ERROR)
    {           
        //check if master slave communication is back.
        if (VCFaultManagment_GetMasterSlaveDetectionFlag())
        { 
            //clear slave lost communication error. 
            VCSTM_FaultProcessing(pHandle, 0, VC_SLAVE_COMM_ERROR);
        }
    } 

    //verify if a VC_SW_ERROR fault was set.
    //this type of error can be cleared inside 
    //of vc faut now state.
    if (pHandle->hVFaultNow & VC_SW_ERROR)
    {           
        //clear VC_SW_ERROR error. 
        VCSTM_FaultProcessing(pHandle, 0, VC_SW_ERROR);
    }  

    //verify if a VC_M1_UNEXPECTED_BEHAVIOR fault was set.
    //this type of error can be cleared inside 
    //of vc faut now state.
    if (pHandle->hVFaultNow & VC_M1_UNEXPECTED_BEHAVIOR)
    {           
        //clear VC_M1_UNEXPECTED_BEHAVIOR error. 
        VCSTM_FaultProcessing(pHandle, 0, VC_M1_UNEXPECTED_BEHAVIOR);
    } 

    //verify if a VC_M2_UNEXPECTED_BEHAVIOR fault was set.
    //this type of error can be cleared inside 
    //of vc faut now state.
    if (pHandle->hVFaultNow & VC_M2_UNEXPECTED_BEHAVIOR)
    {           
        //clear VC_M2_UNEXPECTED_BEHAVIOR error. 
        VCSTM_FaultProcessing(pHandle, 0, VC_M2_UNEXPECTED_BEHAVIOR);
    }  
}

/**
  @brief Function used return the value of the MasterSlaveDetectionFlag.
  @return true is master slave communication is on, false if is off.     
*/
bool VCFaultManagment_GetMasterSlaveDetectionFlag(void)
{
    
    return MasterSlaveDetectionFlag;
}
