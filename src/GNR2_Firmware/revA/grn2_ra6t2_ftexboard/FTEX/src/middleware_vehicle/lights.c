/**
  * @file    lights.c
  * @brief   This module handlesbasic light management
  *
  */

#include "lights.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
 *  Initializes light sensor module
 */
void Light_Init(Light_Handle_t * pHandle)
{	
    ASSERT(pHandle != NULL); 
    struct GPIOConfig PinConfig;
   
    PinConfig.PinDirection = OUTPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(pHandle->wPinNumber, PinConfig);
    
    Light_Disable(pHandle); // We make sure the lights are off on start up
}

/**
 *  Activates the light
 */
void Light_Enable(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if(pHandle->bIsInvertedLogic) // If the logic is inverted we turn on the light with a 0
    {
        uCAL_GPIO_Reset(pHandle->wPinNumber);
    }
    else
    {    
        uCAL_GPIO_Set(pHandle->wPinNumber);
    }
}

/**
 *  Disactivates the light
 */
void Light_Disable(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if(pHandle->bIsInvertedLogic) // If the logic is inverted we turn off the light with a 1
    {
        uCAL_GPIO_Set(pHandle->wPinNumber);
    }
    else
    {     
        uCAL_GPIO_Reset(pHandle->wPinNumber);
    }
}

/**
 *  Toggle the light
 */
void Light_Toggle(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    uCAL_GPIO_Toggle(pHandle->wPinNumber);
}

/**
 *  Check if the light is activated or not
 */
bool Light_GetState(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    return(uCAL_GPIO_Read(pHandle->wPinNumber));
}

