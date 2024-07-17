/**
  * @file    regen.h
  * @brief   This module handles light management
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCREGEN_H
#define __VCREGEN_H

#include "stdbool.h"
#include "stdint.h"

/**
  * @brief Handle used to operate a light
  *
  */
typedef struct
{          
    bool bRegenEnabled;     // Regen enabled state
    uint16_t hMaxCurrent;  // Maximum current
    uint16_t hMinCurrent;   // Minimum current
    uint16_t hRampPercent;  // Ramp percentage
    uint16_t hMaxVoltage;   // Maximum voltage
    uint16_t hMinSpeed;   // Minimum speed  
    uint16_t bRegenLevelPercent;      // Regen level percentage
    float fRampCoEff;       // Ramp coefficient
} Regen_Handle_t;
#endif
