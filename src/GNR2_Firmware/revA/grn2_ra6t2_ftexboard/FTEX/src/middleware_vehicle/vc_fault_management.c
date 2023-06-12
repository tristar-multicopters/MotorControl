/**
*  @file vc_fault_management.c
*  @brief middleware layer used to management VC faults.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "vc_fault_management.h"

/**
 *@brief Function used to get the flag that indicate if
 *       a PDO message was received. THis indicate that 
 *       master and slave communication is on.
 *@return void
*/
extern bool COPGetPdoReceivedFlag(void);

/*********************************************
                Defines
*********************************************/

/*********************************************
                Private Variables
*********************************************/

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
    if (!COPGetPdoReceivedFlag())
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
    
    //verify what kind of fault was triggered.
    if (pHandle->hVFaultNow & VC_STOP_TIMEOUT)
    {
        //clear VC_STOP_TIMEOUT error.     
        //the idea is clear start timeout fault and make the
        //system back to idle state again and try to run again.
        VCSTM_FaultProcessing(pHandle, 0, VC_STOP_TIMEOUT);
    }
    
    //verify if was a lost of communication with
    //slaver.
    if (pHandle->hVFaultNow & VC_SLAVE_COMM_ERROR)
    {           
        //check if VC_SLAVE_COMM_ERROR was clean.
        if (COPGetPdoReceivedFlag())
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

