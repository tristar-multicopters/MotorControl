/**
  * @file    lights.c
  * @brief   This module handlesbasic light management
  *
  */

#include "lights.h"
#include "ASSERT_FTEX.h"

void Light_SetLight(Light_Handle_t * pHandle);
void Light_ResetLight(Light_Handle_t * pHandle);
void Light_Toggle(Light_Handle_t * pHandle);

/* Private Functions -------------------------------------------- */

/**
 * @brief Turns on the light
 * @param pHandle : handle of the light
 */
void Light_SetLight(Light_Handle_t * pHandle)
{
    if (pHandle->bIsInvertedLogic) // If the logic is inverted we turn on the light with a 0
    {
        uCAL_GPIO_Reset(pHandle->PinNumber);
    }
    else
    {    
        uCAL_GPIO_Set(pHandle->PinNumber);
    }
}

/**
 * @brief Turns off the light
 * @param pHandle : handle of the light
 */
void Light_ResetLight(Light_Handle_t * pHandle)
{
    if (pHandle->bIsInvertedLogic) // If the logic is inverted we turn off the light with a 1
    {
        uCAL_GPIO_Set(pHandle->PinNumber);
    }
    else
    {     
        uCAL_GPIO_Reset(pHandle->PinNumber);
    }   
}

/**
 * @brief Toggles the light
 * @param pHandle : handle of the light
 */
void Light_Toggle(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    uCAL_GPIO_Toggle(pHandle->PinNumber);
}

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
    
    uCAL_GPIO_ReInit(pHandle->PinNumber, PinConfig);
}


/**
 *  Sets the desired value of the light when the bike is powered on 
 */
void Light_PowerOnSequence(Light_Handle_t * pHandle)
{
    pHandle->bLightIsActive = pHandle->bDefaultLightState;
}

/**
 *  Turns off the light when the bike is powered off 
 */
void Light_PowerOffSequence(Light_Handle_t * pHandle)
{
    pHandle->bLightIsActive = false;
}

/**
 *  Activates the light
 */
void Light_Enable(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if (!pHandle->bLightStateLocked) // If the lights are not locked to prevent a change
    {    
        pHandle->bLightIsActive = true;
    
        if (!pHandle->bLightIsBlinking) // If we are currently blinking the light we                              
        {                               // dont want to overwrite that.
            Light_SetLight(pHandle);
        }
    }
}

/**
 *  Disactivates the light
 */
void Light_Disable(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if (!pHandle->bLightStateLocked) // If the lights are not locked to prevent a change
    { 
        pHandle->bLightIsActive = false;
    
        Light_ResetLight(pHandle);
    }
}

/**
 *  Check if the light is activated or not
 */
bool Light_GetState(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    return(uCAL_GPIO_Read(pHandle->PinNumber));
}

/**
 *  Function being called periodically to handle the blinking of a light
 */
void Light_Blink(Light_Handle_t * pHandle)
{
    if (pHandle->bLightIsBlinking && pHandle->bLightIsActive)
    {
        if(Light_GetState(pHandle) == true)
        {
            pHandle->BlinkPeriod = pHandle->BlinkPeriodON;
        }
        else
        {
            pHandle->BlinkPeriod = pHandle->BlinkPeriodOFF;
        }   
        
        if (pHandle->BlinkCounter >= pHandle->BlinkPeriod) // If this function has been called enough 
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
        Light_SetLight(pHandle);
    }
    else if (!pHandle->bLightIsActive)
    {
        Light_ResetLight(pHandle);
    }        
}    

/**
 *  Function used activate or deactivate the blinking function of a light
 */
void Light_SetBlink(Light_Handle_t * pHandle, bool BlinkEnable)
{
    pHandle->bLightIsBlinking = BlinkEnable;
}
