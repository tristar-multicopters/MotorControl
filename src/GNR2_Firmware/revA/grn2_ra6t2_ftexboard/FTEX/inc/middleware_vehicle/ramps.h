/**
  * @file    power_enable.h
  * @brief   This module handles power enable pin
  *
*/
    
    /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RAMPS_H
#define __RAMPS_H

#include "ASSERT_FTEX.h"
#include "stdint.h"
#include "stdbool.h"


#define RAMP_TIMESCALE_COUNTER 25        // This determines how often the ramp applies a new setpoint
                                         // It corresponds to multiples of the powertrain loop freq 
                                         // As of the implementation of this code that is 5 ms 
                                          
typedef enum //enum used to specify which type of ramp to be applied.
{
  NO_RAMP,
  LINEAR,
} RAMPS_RampType_t;

typedef enum //enum used to specify which type of ramp to be applied.
{
  ACCELERATION,
  DECELERATION,
} RAMPS_RampDirection_t;

typedef struct
{          
  float Alpha; 
  float MaxDelta;  
} Ramps_Linear_Parameters_t; 

typedef struct
{          
  uint16_t PreviousValue;    // Contains the last value that was computed
  uint16_t RampMax;          // Contains the value where the ramp needs to saturate  
  RAMPS_RampType_t RampType; // Specifies which type of ramp is desired    
  RAMPS_RampDirection_t RampDirection; // Specifies if we need to apply the ramp on an acceleration or deceleration  
  Ramps_Linear_Parameters_t LinearParameters;   
} Ramps_Handle_t;  




/**
  @brief Function used to initialise ramps, calls the proper function for the type of the ramp used
  @param pHandle : Pointer on Handle structure of Ramps module
  @return void
*/
void Ramps_Init(Ramps_Handle_t * pHandle);

/**
  @brief Function used to initialise linear ramps
  @param pHandle : Pointer on Handle structure of Ramps module
  @return void
*/
void Ramps_LinearInit(Ramps_Handle_t * pHandle);

/**
  @brief Function used to apply a ramp
  @param pHandle : Pointer on Handle structure of Ramps module, next value to input in the ramp
  @return void
*/
uint16_t Ramps_ApplyRamp(Ramps_Handle_t * pHandle, uint16_t Input);

/**
  @brief Function used to apply a linear ramp
  @param pHandle : Pointer on Handle structure of Ramps module
  @return void
*/
uint16_t Ramps_ApplyLinearRamp(Ramps_Handle_t * pHandle, uint16_t Input);

/**
  @brief Function used reset a ramp
  @param pHandle : Pointer on Handle structure of Ramps module
  @return void
*/
void Ramps_ResetRamp(Ramps_Handle_t * pHandle);
#endif /*__RAMPS_H*/

