/**
  * @file    lights.h
  * @brief   This module handles light management
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIGHTS_H
#define __LIGHTS_H

#include "stdbool.h"
#include "uCAL_GPIO.h"

/**
  * @brief Handle used to operate a light
  *
  */
typedef struct
{          
    uint32_t wPinNumber;    // Contains the pin number on which the brake is connected
    uint16_t BlinkPeriode;  // How many calls of the blink function need to be called before the led is
                            // toggled. The blinking frequency of the light depends on how quickly
                            // the blink function gets called.
    
    uint16_t BlinkCounter;  // Keeps track of when we need to toggle the LED
	
    bool bLightIsBlinking;  // Shows if the light should be blinking
    bool bLightIsActive;    // Shows if the light should be on or off
    bool bIsInvertedLogic;  // States if the logic is inverted 
	                        // That would mean you would turn the light on with a 0 and off with a 1
} Light_Handle_t;

/**
 * @brief Initializes 
 * @param pHandle : handle of the light
 */
void Light_Init(Light_Handle_t * pHandle);

/**
 * @brief Activates the light
 * @param pHandle : handle of the light
 */
void Light_Enable(Light_Handle_t * pHandle);

/**
 * @brief Disactivates the light
 * @param pHandle : handle of the light
 */
void Light_Disable(Light_Handle_t * pHandle);

/**
 * @brief Toggles the light
 * @param pHandle : handle of the light
 */
void Light_Toggle(Light_Handle_t * pHandle);

/**
 * @brief Check if the light is activated or not
 * @param pHandle : handle of the light
 */
bool Light_GetState(Light_Handle_t * pHandle);

/**
 * @brief Function being called periodically to handle the blinking of a light
 * @param pHandle : handle of the light
 */
void Light_Blink(Light_Handle_t * pHandle);

/**
 * @brief Function used activate or deactivate the blinking function of a light
 * @param pHandle : handle of the light
 */
void Light_SetBlink(Light_Handle_t * pHandle, bool BlinkEnable);

#endif /*__LIGHTS_H*/

