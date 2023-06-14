/**
* @file   vc_autodetermination.h
* @author Bruno Alves
* @brief  Application layer used to 
*         determine device function(master or slave).
*
*
*/

#ifndef VC_AUTODETERMINATION_H_
#define VC_AUTODETERMINATION_H_

/*********************************************
               Includes                       
*********************************************/

#include "power_enable.h"

/*********************************************
                Defines
*********************************************/


/*********************************************
          Data Struct Definition
*********************************************/


//========================= EXTERN TYPES ==========================//


// ==================== Public function prototypes ========================= //
/**
  @brief Function used to read the device function, master or slave.
 */
bool VcAutodeter_GetGnrState(void);

/**
  @brief Function used to set the device function to master.
*/
void VcAutodeter_SetGnrMaster(void);

/**
  @brief Function used to set the device function to slave.
*/
void VcAutodeter_SetGnrSlave(void);

/**
  @brief Function used set the GNR function/state(master or slave).
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
*/
void VcAutodeter_MasterSlaveDetection(PWREN_Handle_t * pHandle);

#endif