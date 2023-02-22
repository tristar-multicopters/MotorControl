/**
  * @file    HardwareOverCurrentDetection.c
  * @author  Behnam Shakibafar, FTEX
  * @brief   This module handles Hardware Over Current Detection signal management
  *
*/

#include "HardwareOverCurrentDetection.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
 *  Initializes OCD2 sensor module
 */
void OCD2_Init(OCD2_Handle_t * pHandle)
{	
    ASSERT(pHandle != NULL); 
    struct GPIOConfig pinConfig;
   
    pinConfig.PinDirection = INPUT;
    pinConfig.PinPull      = UP; 
    pinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(pHandle->wPinNumber, pinConfig);
}

/**
 *  Returns state of the OCD2
 */
bool OCD2_IsEnabled(OCD2_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL); 
    bool bAux = uCAL_GPIO_Read(pHandle->wPinNumber);
    pHandle->bOCD2IsEnabled = !bAux;        

    return pHandle->bOCD2IsEnabled;
}


