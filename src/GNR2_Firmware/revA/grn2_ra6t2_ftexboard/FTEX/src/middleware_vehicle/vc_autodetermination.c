/**
*  @file vc_autodetermination.c
*  @brief Application layer used to auto detect device
          functionality(master or slave).
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "vc_autodetermination.h"

/*********************************************
                Defines
*********************************************/

/*********************************************
                Private Variables
*********************************************/
//variable used to know if the device will be configured as master
//or slave.
static bool masterDevice = false;
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
  Function used to read the device function, master or slave.
  must to be used after device state was detected.
 */
bool VcAutodeter_GetGnrState(void)
{
    return masterDevice;
}

/**
  @brief Function used to set the device function to master.
*/
void VcAutodeter_SetGnrMaster(void)
{
    masterDevice = true;
}

/**
  @brief Function used to set the device function to slave.
*/
void VcAutodeter_SetGnrSlave(void)
{
    masterDevice = false;
}

/**
  @brief Function used set the GNR function/state(master or slave).
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
*/
void VcAutodeter_MasterSlaveDetection(PWREN_Handle_t * pHandle)
{
    //verify the pointers.
    ASSERT(pHandle != NULL);
    
    //if device must to communicate
    //with an IOT, it must to be master.
    #if GNR_IOT == 1
    //set master function true.
    VcAutodeter_SetGnrMaster(); 
    #else
    //verify the pwr lock signal is acitve(indicate GRN must be configured
    //as a master).
    if (PWREN_PWRDetected(pHandle))
    {
        //set master function true.
        VcAutodeter_SetGnrMaster();
    }
    else
    {
        //set master function false.
        VcAutodeter_SetGnrSlave();
    }
    #endif
}

