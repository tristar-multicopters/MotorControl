/**
  * @file    brake.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles brake sensor management
  *
*/

#include "brake.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
 *  Initializes brake sensor module
 */
void BRK_Init(BRK_Handle_t * pHandle)
{	
   ASSERT(pHandle != NULL); 
   struct GPIOConfig PinConfig;
   
   PinConfig.PinDirection = INPUT;
   PinConfig.PinPull      = UP; 
   PinConfig.PinOutput    = PUSH_PULL; 
    
   uCAL_GPIO_ReInit(pHandle->wPinNumber, PinConfig);
}

/**
 *  Returns state of the brake
 */
bool BRK_IsPressed(BRK_Handle_t * pHandle)
{
	ASSERT(pHandle != NULL); 
    bool bAux = uCAL_GPIO_Read(pHandle->wPinNumber);
	pHandle->bIsPressed = bAux ^ pHandle-> bIsInvertedLogic;
	
	return pHandle->bIsPressed;
}

