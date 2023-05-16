/**
* @file   firmware_update.h
* @author Bruno Alves
* @brief  Application layer used to handle DFU process.
*
*
*/

#ifndef VC_FAULT_MANAGEMENT_H_
#define VC_FAULT_MANAGEMENT_H_

/*********************************************
               Includes                       
*********************************************/

#include "ASSERT_FTEX.h"
#include "co_gnr2_specs.h"
#include "vc_state_machine.h"
#include "power_enable.h"

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
 *@brief Function used to set the presence of the device, master or slave.
 *       It send a sdo download command and if the other side responde a flag
 *       is set to indicate that command was received.
 *       No sdo response, clear the correspondent flag(MasterSlaveDetectionFlag).
 *@param pNode Pointer on the CANOpen node to operate on.
 *@param void.
*/
void VCFaultManagment_MasterSlaveDetection(CO_NODE  *pNode, PWREN_Handle_t * pHandle);

/**
  @brief SDO transfer finalization callback function used when
         sending sdodownload command to master/slave module.
  @param CO_CSDO *csdo Pointer to SDO client object
  @param uint16_t index address of the sdo service.
  @param uint8_t sub subindex number of the sdo service.
  @param uint32_t code error code from the SDO service.
  @return void
*/
void MasterSlaveDetection_CallbackSDODownloadFinish(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code);

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