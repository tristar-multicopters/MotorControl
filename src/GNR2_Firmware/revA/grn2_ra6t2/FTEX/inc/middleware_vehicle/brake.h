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
} BRK_Handle_t;

/**
 * @brief Initializes Brake module pin
 * @param pHandle : Pointer on Handle structure of Brake module
 */
void BRK_Init( BRK_Handle_t * pHandle);

/**
 * @brief Checks if the brake is pressed
 * @param pHandle : Pointer on Handle structure of Brake module
 * @return state of the brake
 */
bool BRK_IsPressed( BRK_Handle_t * pHandle);


#endif /*__BRAKE_H*/

