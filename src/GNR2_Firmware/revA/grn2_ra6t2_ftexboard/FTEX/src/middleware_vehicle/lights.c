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
    
    pHandle->bLightIsActive = true;
    
    if(!pHandle->bLightIsBlinking) // If we are currently blinking the light we                              
    {                              // dont want to overwrite that.
        
        if(pHandle->bIsInvertedLogic) // If the logic is inverted we turn on the light with a 0
        {
            uCAL_GPIO_Reset(pHandle->wPinNumber);
        }
        else
        {    
            uCAL_GPIO_Set(pHandle->wPinNumber);
        }
    }
}

/**
 *  Disactivates the light
 */
void Light_Disable(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);

    pHandle->bLightIsActive = false;
    
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

/**
 *  Function being called periodically to handle the blinking of a light
 */
void Light_Blink(Light_Handle_t * pHandle)
{
    if (pHandle->bLightIsBlinking && pHandle->bLightIsActive)
    {
        if(pHandle->BlinkCounter >= pHandle->BlinkPeriode) // If this function has been called enough 
        {                                                  // so that Blink Counter equals Blink Periode   
            pHandle->BlinkCounter = 0; // reset the counter
            Light_Toggle(pHandle);     // Toggle the light
        }
        else
        {
            pHandle->BlinkCounter ++;
        }
    }
    else if (pHandle->bLightIsActive) //If we arent blinking keep the light state
    {
        Light_Enable(pHandle);
    }
    else if (!pHandle->bLightIsActive)
    {
        Light_Disable(pHandle);
    }        
}    

/**
 *  Function used activate or deactivate the blinking function of a light
 */
void Light_SetBlink(Light_Handle_t * pHandle, bool BlinkEnable)
{
    pHandle->bLightIsBlinking = BlinkEnable;
}
