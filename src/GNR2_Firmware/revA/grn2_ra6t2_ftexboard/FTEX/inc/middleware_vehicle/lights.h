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


#endif /*__LIGHTS_H*/

