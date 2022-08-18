/**
  ******************************************************************************
  * @file    pulse_frequency.h
  * @author  FTEX inc
  * @brief   Header for the Pulse frequency reading module
	*					 pulse_frequency_gpt.c using General Purpose Timer
  ******************************************************************************
*/

#ifndef __PULSE_FREQUENCY_H
#define __PULSE_FREQUENCY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "r_gpt.h"
#include "r_agt.h"

typedef enum
{
	AGT_TIMER = 0,
	GPT_TIMER,
} TimerType_t;

typedef struct
{
	TimerType_t TimerType;
    const timer_instance_t* pTimer;
	
    uint32_t wCaptureCount; 	/* GPT timer capture variable */
    uint32_t wCaptureOverflow;  /* GPT timer capture overflow variable */
	
} PulseFrequencyHandle_t; 



// ==================== Public function prototypes ========================= //

/**
  @brief  Function used to Initialize the timer for Capture Mode 
  @param  PulseFrequencyHandle_t
  @return bIsError in boolean
*/
bool PulseFrequency_Init(PulseFrequencyHandle_t * pHandle);

/**
  @brief  Function used to return frequency of pulse signal
  @param  PulseFrequencyHandle_t handle
  @return Latest frequency measurement of pulse signal
*/
uint32_t PulseFrequency_GetFrequency(PulseFrequencyHandle_t * pHandle);

/**
  @brief  Function used to return period of pulse signal
  @param  PulseFrequencyHandle_t handle
  @return Latest period measurement of pulse signal
*/
uint32_t PulseFrequency_GetPeriod(PulseFrequencyHandle_t * pHandle);

/**
  @brief  Function used to update the capture variables from the interrupt
  @param  PulseFrequencyHandle_t handle
  @return None
*/
void PulseFrequency_ISRCaptureUpdate(PulseFrequencyHandle_t * pHandle , uint32_t  wCapture);

/**
  @brief  Function used to update the overflow variable from the interrupt
  @param  PulseFrequencyHandle_t handle
  @return None
*/
void PulseFrequency_ISROverflowUpdate(PulseFrequencyHandle_t * pHandle);
	

#endif /* __PULSE_FREQUENCY_H */

