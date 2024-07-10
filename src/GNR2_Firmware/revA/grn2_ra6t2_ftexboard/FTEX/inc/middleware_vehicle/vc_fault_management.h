/**
* @file   vc_faut_management.h
* @author Bruno Alves
* @brief  Application layer used to handle vc faults.
*
*
*/

#ifndef VC_FAULT_MANAGEMENT_H_
#define VC_FAULT_MANAGEMENT_H_

/*********************************************
               Includes                       
*********************************************/
#include "co_gnr2_specs.h"
#include "vc_state_machine.h"
/*********************************************
                Defines
*********************************************/
//this define is used in one function called
//every 25 ms.
#define MASTERSLAVE_DETECTION_TIME_250MS 9

//this define is used in one function called
//every 25 ms.
#define MASTERSLAVE_LOSTDETECTION_TIMEOUT_500MS 20

//timeout when waiting for one sdo download response.
#define SDODOWNLOAD_TIMEOUT_MS   50

/*********************************************
          Data Struct Definition
*********************************************/


//========================= EXTERN TYPES ==========================//


// ==================== Public function prototypes ========================= //

/**
  @brief Function used to detect if slave or master are not present anymore.
         This means more than 0.5 seconds without communication. 
         This function is verified every 25 ms.
  @param bool true if the communication between master/slave was lost.
*/
bool VCFaultManagment_MasterSlaveCommunicationLost(void);

/**
  @brief Function used to process faults happened in the VC layer.
         If fault condition finished, clear fault error on vc layer.
  @param VCSTM_Handle_t *pHandle pointer to the struct responsible to handle VC faults.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @return void.
*/
void VCFaultManagment_Processing(VCSTM_Handle_t *pHandle, CO_NODE  *pNode);

#endif