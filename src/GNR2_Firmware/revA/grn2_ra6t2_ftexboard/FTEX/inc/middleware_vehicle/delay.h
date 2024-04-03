/**
  * @file    Delay.h
  * @brief   This module handles delays based on counters
  *
  */
    
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Delay_H
#define __Delay_H

#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"

// Contaisn the various time units that are supported by the delay module
typedef enum
{   
    MIC_SEC = 1, // Microseconds
    MIL_SEC = 2, // Milliseconds
        SEC = 3, // Seconds

}DelayUnits_t;

/**
  * @brief Generic structure containing the information inside the handle of a delay 
  */
typedef struct
{                  
    uint32_t TimePerPulse;       // States how much time per pulse ex if 1 pulse is 5 ms then thos variable wpuld be 5
    DelayUnits_t TimePerPulseUnits;  // States in which units os the time per pulse, if you hate 5 ms then this would be set to MIL_SECS 
    
    uint32_t DelayTimePulse;     // Represents how many pulse we need to wait for to achieve the desired Delay
    uint32_t DelayCounter;       // Keeps track of the Delay count 
    
    bool DelayInitialized;  
    
} Delay_Handle_t;


/**
  * @brief  Initializes Delay
  * @param  pHandle : specific isntance of a delay, aPulseTime the length of a simngle pulse, pulse unit : the unit associated with a pulse time
  * @retval Nothing  
  */
void Delay_Init(Delay_Handle_t * pHandle, uint32_t aPulseTime, DelayUnits_t aPulseUnit);

/**
  * @brief  Update the delay 
  * @param  pHandle : specific isntance of a delay
  * @retval bool, true if the delay has overflown, false if not
  */
bool Delay_Update(Delay_Handle_t * pHandle);

/**
  * @brief  Reset the delay
  * @param  pHandle : specific isntance of a delay
  * @retval Nothing
  */
void Delay_Reset(Delay_Handle_t * pHandle);

/**
  * @brief  Set the time of delay
  * @param  pHandle : specific isntance of a delay, aDelay : set the amount of time until we overflow, aDelayUnits set the unit associated with the Delay
  * @retval Nothing
  */
void Delay_SetTime(Delay_Handle_t * pHandle, uint32_t aDelay, DelayUnits_t aDelayUnits);

#endif /*__DELAY_H*/

