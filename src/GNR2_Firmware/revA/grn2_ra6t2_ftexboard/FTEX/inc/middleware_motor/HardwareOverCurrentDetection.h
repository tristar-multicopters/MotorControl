/**
  * @file    HardwareOverCurrentDetection.h
  * @author  Behnam Shakibafar, FTEX
  * @brief   This module handles Hardware Over Current Detection signal management
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWAREOVERCURRENTDETECTION_H
#define __HARDWAREOVERCURRENTDETECTION_H

#include "stdbool.h"
#include "uCAL_GPIO.h"
#include "board_hardware.h"
#include "r_poeg.h"
#include "hal_data.h"


/**
  * @brief OCD2_Handle_t structure used for brake sensing
  *
  */
typedef struct
{          
	bool bOCD2IsEnabled;    // Contains the actual state of the OCD2
    uint32_t wPinNumber;    // Contaisn the pin number on which the OCD2 is connected
    bool bIsInvertedLogic;  // States if the logic is inverted 
} OCD2_Handle_t;

/**
 * @brief Initializes Brake module pin
 * @param pHandle : Pointer on Handle structure of Brake module
 */
void OCD2_Init(OCD2_Handle_t * pHandle);

/**
 * @brief Checks if the brake is pressed
 * @param pHandle : Pointer on Handle structure of OCD2 module
 * @return state of the brake
 */
bool OCD2_IsEnabled(OCD2_Handle_t * pHandle);

#endif /*__BRAKE_H*/

