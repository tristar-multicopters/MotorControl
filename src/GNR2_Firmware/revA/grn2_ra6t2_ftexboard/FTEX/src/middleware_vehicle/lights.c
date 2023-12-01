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
void Light_ApplyState(Light_Handle_t * pHandle);
/* Private Functions -------------------------------------------- */

/**
 * @brief Turns on the light
 * @param pHandle : handle of the light
 */
void Light_SetLight(Light_Handle_t * pHandle)
{
    if (pHandle->bIsInvertedLogic) // If the logic is inverted we turn on the light with a 0
    {
        uCAL_GPIO_Reset(pHandle->wPinNumber);
    }
    else
    {    
        uCAL_GPIO_Set(pHandle->wPinNumber);
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
        uCAL_GPIO_Set(pHandle->wPinNumber);
    }
    else
    {     
        uCAL_GPIO_Reset(pHandle->wPinNumber);
    }   
}

/**
 * @brief Applies the current state to the light pin
 * @param pHandle : handle of the light
 */
void Light_ApplyState(Light_Handle_t * pHandle) // Used to apply the stae of the module to the pin
{
    if (pHandle->bLightIsActive) 
    {
        Light_SetLight(pHandle);
    }
    else if (!pHandle->bLightIsActive)
    {
        Light_ResetLight(pHandle);
    }
}  

/**
 * @brief Toggles the light
 * @param pHandle : handle of the light
 */
void Light_Toggle(Light_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    uCAL_GPIO_Toggle(pHandle->wPinNumber);
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
    
    uCAL_GPIO_ReInit(pHandle->wPinNumber, PinConfig);
    
    pHandle->bInternalUpdateFlag = false;
}


/**
 *  Sets the desired value of the light when the bike is powered on 
 */
void Light_PowerOnSequence(Light_Handle_t * pHandle)
{
    pHandle->bLightIsActive = pHandle->bDefaultLightState;
    
    Light_ApplyState(pHandle); 
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
    
    if (!pHandle->bLightStateLocked && pHandle->bLightIsActive != true) // If the lights are not locked to prevent a change
    {    
        pHandle->bLightIsActive = true;
        pHandle->bInternalUpdateFlag = true;
        
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
    
    if (!pHandle->bLightStateLocked && pHandle->bLightIsActive != false) // If the lights are not locked to prevent a change
    { 
        pHandle->bLightIsActive = false;
    
        Light_ResetLight(pHandle);
        pHandle->bInternalUpdateFlag = true;
    }
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
        if (pHandle->BlinkCounter >= pHandle->BlinkPeriode) // If this function has been called enough 
        {                                                  // so that Blink Counter equals Blink Periode   
            pHandle->BlinkCounter = 0; // reset the counter
            Light_Toggle(pHandle);     // Toggle the light
        }
        else
        {
            pHandle->BlinkCounter ++;
        }
    }
    else if (pHandle->bOldBlinkState && !pHandle->bLightIsBlinking) //If we arent blinking but we were last time restore the light state
    {
        Light_ApplyState(pHandle);
    }
    
    pHandle->bOldBlinkState = pHandle->bLightIsBlinking;
}    

  


/**
 *  Function used activate or deactivate the blinking function of a light
 */
void Light_SetBlink(Light_Handle_t * pHandle, bool BlinkEnable)
{
    pHandle->bLightIsBlinking = BlinkEnable;
}

/**
 *  Function used activate or deactivate the blinking function of a light due to the brake being pressed
 */
void Light_SetBlinkByBrake(Light_Handle_t * pHandle, bool BlinkEnable)
{
    
    if(pHandle->bBlinkOnBrake)
    {        
        pHandle->bLightIsBlinking = BlinkEnable;
    }
}

// Check if we had an internal change of the light state
bool Light_CheckInternalUpdateFlag(Light_Handle_t * pHandle)
{
    return pHandle->bInternalUpdateFlag;
}

// Clear the internal update flag after processing the change in CAN 
void Light_ClearInternalUpdateFlag(Light_Handle_t * pHandle)
{
    pHandle->bInternalUpdateFlag = false;
}