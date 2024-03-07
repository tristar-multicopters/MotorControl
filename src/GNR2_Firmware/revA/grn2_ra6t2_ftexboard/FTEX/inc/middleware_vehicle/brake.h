/**
  * @file    brake.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles brake sensor management
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BRAKE_H
#define __BRAKE_H

#include "stdbool.h"
#include "uCAL_GPIO.h"
#include "vc_errors_management.h"
#include "delay.h"

// ============================= Defines ================================= //

#define SAFE_BRAKE_COUNT_100MS           (uint16_t)20 // Called every 5ms, 20*5 = 100ms, which is throttle settle time

/**
  * @brief Brake_Handle_t structure used for brake sensing
  *
  */
typedef struct
{          
    bool bIsPressed;        // Contains the actual state of the brake
    uint32_t wPinNumber;    // Contaisn the pin number on which the brake is connected
    
    bool bIsInvertedLogic;  // States if the logic is inverted 
                            // If set to true then a 1 would mean that the brake isnt 
                            // pressed and a 0 would mean it is pressed
    bool bEngagedOnStart; // Is the brake live on boot 
    bool bSafeStart;      // brake value has been measured safely

    Delay_Handle_t * pBrakeStuckDelay;

} BRK_Handle_t;

/**
 * @brief Initializes Brake module pin
 * @param pHandle : Pointer on Handle structure of Brake module
 */
void BRK_Init(BRK_Handle_t * pHandle, Delay_Handle_t * pBrakeDelay);

/**
 * @brief Checks if the brake is pressed
 * @param pHandle : Pointer on Handle structure of Brake module
 * @return state of the brake
 */
bool BRK_IsPressed(BRK_Handle_t * pHandle);

/**
 * @brief Checks if the brake is pressed and if it's stuck
 *        MUST ONLY BE CALLED IN ONE PLACE
 *
 * @param pHandle : Pointer on Handle structure of Brake module
 * @return state of the brake
 */
bool BRK_IsPressedSafety(BRK_Handle_t * pHandle);

#endif /*__BRAKE_H*/

